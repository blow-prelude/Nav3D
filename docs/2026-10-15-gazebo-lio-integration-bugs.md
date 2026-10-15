# Gazebo 仿真 × LIO 联调问题记录（2026-10-15）

> 范围：GazeboQuadbot（仿真底座）与 Nav3D/src/lio（Elevator-LIO）首次联调"建图 → RViz 可视化"全链路。
> 所有条目均已修复并验证。涉及提交：
> - Nav3D：`7ecc022`（仿真配置）、`4b37736`（rviz RobotModel）、`6c6c20e`（时钟节流修复）
> - GazeboQuadbot：`e16bb63`（点云时间戳 + 量程）、`d98a70b`（mesh file:// 前缀）

## 现象回顾

| 阶段 | 表象 |
|---|---|
| 初次启动 LIO | rviz 全黑，tf tree 只有机器人自身 TF，无 odom |
| 修复类型后 | rviz 只显示狗身体和雷达支架，四条腿缺失 |
| 修复节流后 | 腿有了，但报 `Could not load mesh resource ... livox_frame.STL` |

三个阶段表象不同，根因层层叠加，共 10 个独立问题。

## 问题总表

| # | 问题 | 类别 | 一句话根因 |
|---|---|---|---|
| A1 | LIO 收不到任何点云（静默） | 接口不匹配 | `lidar_type: 1` 订阅 Livox CustomMsg，仿真发的是 PointCloud2 |
| A2 | rviz 加载不了 livox 网格 | 接口不匹配 | mesh 裸绝对路径无 `file://` scheme，Gazebo 认、RViz 不认 |
| B1 | 建图 0.83Hz、TF 落后分钟级、模型只剩"身体" | 时钟时域 | lio FSM 绑定 /clock（仅 ~10Hz），IMU 消费速率坍缩 |
| C1 | 点云帧时间漂移、去畸变失效 | 仿真实现 | 插件每点时间戳 = 绝对仿真时间 + CSV 行号，非帧内相对时间 |
| C2 | 建图场景受限 | 仿真实现 | 仿真雷达量程仅 5m（Mid-360 实际 70m） |
| D1 | 仿真 gzserver 被误杀 | 环境/进程 | 残留调试脚本末尾的 `pkill -x gzserver` |
| D2 | 可能重复 spawn 狗 | 环境/进程 | 孤儿 spawn_entity 等待已死的 Gazebo 服务 |
| D3 | 双 lio 实例打架 | 环境/进程 | 停止 `ros2 launch` 未杀子进程，旧实例继续发布同一 TF |
| E1 | 照 README 操作必失败 | 文档资产 | 引用仓库里不存在的 `root_config_sim.yaml`/`mid360_sim.yaml`、旧机器路径 |
| E2 | 外参按 README 推导会错 | 文档资产 | 外参 R 矩阵转置约定搞混（见 A1 修复详述） |
| E3 | LIO rviz 永远没有狗模型 | 可视化配置 | rviz 配置无 RobotModel 显示项 |

---

## A. 接口/协议不匹配类

### A1 雷达消息类型不匹配（静默失败）

- **现象**：LIO 正常启动、无任何报错，但永不输出 TF/地图。
- **根因**：默认配置 `root_config.yaml → sensors/livox.yaml` 为 `lidar_type: 1`（Livox CustomMsg）；Gazebo mid360 插件发布 `sensor_msgs/PointCloud2`。ROS 2 中同话题不同类型**不报错、不连接**；且 pc2 回调里 `case Livox: break;` 即使收到也会丢弃。
- **修复**（Nav3D `7ecc022`）：新增 `yaml/sensors/mid360_sim.yaml`（`lidar_type: 2`）+ `root_config_sim.yaml` + `runtime/mapping_sim.yaml`。
- **教训**：ROS 2 类型不匹配是**完全静默**的。判断数据链路是否接通，永远以 `ros2 topic info <topic> -v` 里发布者/订阅者类型一致且计数增长为准，不以"节点没报错"为准。

### A2 mesh 裸路径 vs `file://` scheme

- **现象**：rviz RobotModel 报 `Could not load mesh resource '/home/.../install/.../livox_frame.STL'`，文件实际存在。
- **根因**：`mid360.xacro` 中 mesh 写作 `filename="$(find ...)/..."`，xacro 展开后是裸绝对路径。Gazebo 的网格管理器接受裸路径，RViz 的 `resource_retriever` 只认 `file://`/`package://` 等 scheme。狗腿 mesh 恰好都带了 `file://`，所以只有雷达缺。
- **修复**（GazeboQuadbot `d98a70b`）：visual + collision 两处补 `file://` 前缀。
- **教训**：同一资源 URI 会被多个消费者解析（Gazebo、RViz、moveit…），写 URDF 时统一 `file://$(find pkg)/...` 形式，别依赖"恰好这个消费者能容错"。

---

## B. 时钟时域类

### B1 lio FSM 被 /clock 节流（本轮核心 bug）

- **现象**：建图从 10Hz 掉到 **0.83Hz**；`odom→base_link` 时间戳落后仿真时间**分钟级**且永远追不上；rviz 里模型只剩 `/tf_static` 可解析的部分（身体、雷达支架），四条腿（`/tf` 动态变换）全部缺失。CPU 仅 14% —— 不是算力问题。
- **根因链条**：
  1. lio 的处理循环是 FSM 状态机（`Initializing → IMUProcess ⇄ LidarProcess`），由 2000Hz ROS 定时器推进，**每次触发只从 IMU 缓冲弹一条消息**；
  2. 一帧雷达（0.1s）需要消费 ~20 条 IMU 才能满足 `lastest_imu_time >= frame.end_time` 的触发条件；
  3. `use_sim_time:=true` 把定时器绑到 `/clock`，而 Gazebo 的 `/clock` 只有 **~10Hz**（RTF 0.78 下实测 7.8Hz）；
  4. 于是 IMU 消费速率 ≈ 时钟频率量级（实测 **17.7 条/s** vs 到达速率 200Hz），帧率 = 消费速率 ÷ 20 ≈ 0.83Hz。**算术对账：17.7 ≈ 2 × 7.8，0.83 ≈ 17.7/20，完全吻合。**
- **修复**（Nav3D `6c6c20e`）：lio 节点保持墙钟（其流水线完全由消息时间戳驱动，定时器只负责推 FSM，这与 bag 回放模式天然一致）；`use_sim_time` 参数只传给 rviz（显示时钟）。
- **验证**：修复后 7.75Hz = 输入速率（RTF 0.78 下即实时），TF 滞后 ~0.2s。
- **原则**：`use_sim_time` 适合**显示与服务类**节点（rviz、rosbag、TF 消费者）；**由消息时间戳驱动的算法流水线**不要绑 /clock —— 否则算法吞吐被时钟发布频率锁死，且 Gazebo 的 /clock 频率没有任何保证。

---

## C. 仿真实现类

### C1 插件点云时间戳不是帧内相对时间

- **现象**（潜在，联调时被 A1 掩盖）：`lidar_type: 2` 修复后若不修此项，帧 `end_time` 会随扫描周期漂移 0~0.8s，去畸变完全失效，走动时地图拖影。
- **根因**：插件写入每点 `timestamp` 字段的值 = `仿真时间 + CSV 行号(1..800000)`。LIO 的通用 pc2 解析器把该字段经量纲启发式转成"帧内偏移毫秒"，绝对时间+行号进入该管道全是垃圾值。
- **修复**（GazeboQuadbot `e16bb63`）：Load 阶段把 CSV 时间列（采样序号）归一化为帧内相对时间 `[s]`：`fmod(t, samples) / (帧点数 × 更新率)`。
- **教训**：点云无序场里"每点时间"的约定：Livox CustomMsg 为帧内偏移 ms、Velodyne `time` 为帧内偏移 s、Ouster `t` 为 ns。自定义插件要么遵循其中一种，要么在接收端文档里写死约定——不能凭直觉填绝对时间。

### C2 雷达量程 5m

- **修复**（GazeboQuadbot `e16bb63`）：`laser_max_range` 5.0 → 70.0（Mid-360 规格）。
- **备注**：量程增大后单帧点数上升（对 LIO 的 pre-downsample 有压力），但自适应降采样目标 20000 pts/s 不变，实测负载可接受。

---

## D. 环境/进程管理类

### D1 残留调试脚本误杀 gzserver

- **现象**：正在跑的仿真毫无征兆死亡；lio/rviz 随后集体"失明"。
- **根因**：此前调试留下的后台脚本（自起 gzserver → spawn → 结尾 `pkill -x gzserver`）残留在进程表中，其收尾 pkill 波及了用户会话的 gzserver。
- **教训**：留后台实验进程时用 `timeout` 包住或确保自清理；凡带 `pkill`/`killall` 的脚本绝不能裸跑后台。

### D2 孤儿 spawn_entity

- **现象**：Gazebo 已死，`spawn_entity.py` 仍挂着等服务出现。
- **风险**：下次 Gazebo 一起来，孤儿会跟着再 spawn 一只重复的狗。
- **处置**：`pgrep -af spawn_entity` 后 kill。

### D3 僵尸 lio 实例 → 同一 TF 双发布

- **现象**：tf2_echo 里 `odom→base_link` 时间戳在两条时间轴上跳动；rviz 解析不稳定。
- **根因**：停止 `ros2 launch`（或工具层 TaskStop）只杀 launch 进程本身，**已启动的节点子进程会孤儿化继续运行**。旧实例沿旧仿真时间轴发布，与新实例冲突。
- **处置/预防**：重启 lio 前先 `pgrep -af "lio/lib/lio"` 确认唯一；用进程组方式杀（`kill -- -PGID`）或 kill 全部匹配进程。

---

## E. 配置/文档资产类

### E1 README 引用不存在的文件与旧机器路径

- `root_config_sim.yaml`、`yaml/sensors/mid360_sim.yaml` 被引用但从未提交；`/media/lenovo/disk/...` 为旧机器路径。
- **修复**：文件补齐（`7ecc022`），README 教程路径/话题名更新为本机实际值。

### E2 外参转置约定笔误

- README 计划值 `lidar_R_body = R_y(0.4)`、`lidar_t_body = (-0.2806, 0, -0.0224)` 基于 B←L 旋转约定；代码语义（`common_lib.cpp` 注释 + `publish.cpp` 组合 `world_T_imu · imu_T_lidar · lidar_T_body`）要求 **L←B**（body 在 lidar 系下的位姿）。
- 正确值：`lidar_R_body = R_y(-0.4)`、`lidar_t_body = (-0.1796, 0, -0.2171)`（由 URDF 安装 `xyz 0.25 0 0.13, rpy 0 0.4 0` 求逆）。
- **教训**：外参四元组（`*_R_*`/`*_t_*`）每个仓库约定都可能相反，写配置前先读消费代码的注释和组合式，不要照抄其他项目的记法。

### E3 LIO rviz 配置无 RobotModel

- 修复（`4b37736`）：增加 RobotModel 显示项，订阅仿真侧 `robot_state_publisher` 的 `/robot_description`。

---

## 排查方法论（本轮实际用到、可复用）

1. **静默失败三板斧**：`ros2 topic info <t> -v`（类型与连接数）→ `ros2 topic hz <t>`（是否在流）→ `pgrep -af`（实例唯一性）。ROS 2 不会替你报"类型不匹配"。
2. **TF 分段排查**：`view_frames` 看拓扑连通 → `tf2_echo a b` 逐段测（静态帧任何时刻可解析，动态帧受时间有效性约束）——"静态的显示、动态的不显示"＝时间域或新鲜度问题，不是树断了。
3. **吞吐问题用日志对账，不靠猜**：lio 每次运行在 `src/lio/temp/<启动时间>/` 留痕。`state/post_state.csv` 帧距与行数 → 处理节奏；`debug/elevator_debug.csv`（每 5 个 IMU 周期一条）→ FSM 消费 IMU 的真实速率。本轮即由此得出"17.7 条/s ≈ 2×/clock"的关键证据。
4. **算术闭环**：0.83Hz × 20 IMU/帧 ≈ 17 条/s ≈ 2×7.8Hz(/clock)。瓶颈假设能被两个独立观测的乘除关系互相锁死，才算坐实。
5. **改仿真参数后区分生效路径**：xacro 是 symlink 安装（改完即生效，但 URDF 在仿真启动时生成，需重启仿真）；C++ 插件需 `colcon build`；LIO 的 yaml/launch 同为 symlink。别把"没生效"误判为"修错了"。

## 遗留风险与建议

- **Gazebo RTF ≈ 0.78**，gzserver 单核打满（20k rays/帧的 ODE multiray 偏重）。LIO 已不受影响，但实时性敏感的下游（局部规划）联调时需留意仿真时间与墙钟的换算。
- **`Nav3D/GazeboQuadbot` 子模块克隆**停在 `f58d084`，不含 GazeboQuadbot 的两个修复提交（`e16bb63`、`d98a70b`）。切到子模块那份跑仿真前必须先同步；父仓库引用子模块 commit 时也应在 `e16bb63` 之后。
- **仿真重启 = 时间轴重置**，lio 必须跟着重启（其初始化时刻与消息时间轴绑定），否则帧会被判为超出 IMU 时间范围而丢弃。
- `mid360_sim.yaml` 仅属仿真链路；实机/实机 bag 继续用 `mid360_real.yaml` / `mid360_bag.yaml`，不要共用。
