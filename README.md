# Go2W 室内 3D 定位导航避障

目标：在 `/media/lenovo/disk/planner_ws/src/Nav3D/` 单 git 仓库内，参考一个已有的完整的导航栈的结构排步（本地地址为：`/media/lenovo/disk/Embodied_AI/dddmr_navigation`），深度重构 Elevator-LIO / SCAN-Planner / livox_ros_driver2 / jie_octomap 到`Nav3D/src`中。在 Gazebo 和 实机部署中跑通"建图  → 定位 → 全局规划 → 局部避障 → 仿真狗到达"的完整闭环。

包含：

建图`Nav3D/src/lio`（clone https://github.com/xiaofan4122/Elevator-LIO.git ）

全局规划`Nav3D/src/global_planner`（参考 /media/lenovo/disk/3D_Nav/jie_3d_nav 和 /media/lenovo/disk/3D_Nav/OctoPlanner3D-ROS2中的3D A*）

局部规划`Nav3D/src/local_planner`（clone https://github.com/wuyi2121/SCAN-Planner.git）

仿真基座直接使用`/home/ww/program/projects/go2w_outdoor_3d/GazeboQuadbot`

驱动:Mid360驱动使用官方的[`livox_ros_driver2`](https://github.com/Livox-SDK/livox_ros_driver2.git)

分层启动由 `bringup` 编排



仓库构建时指南：

```bash
mkdir [path/planner_ws/src]
cd [path/planner_ws/src]
git clone git@github.com:VisionTrekker/Nav3D.git
```



## Global Constraints

- **ROS 2 发行版**:仅支持 Humble (Ubuntu 22.04)
- **tf frame**: `map → odom → base_link → lidar_frame`(LIO 内部 `world`/`IMU`/`body`/`lidar` 经 C++ `#define` 重命名为 `odom`/`base_link`/`base_link`/`lidar_frame`)
- **话题命名**:**per-module 子命名空间**(`/lio/mapping/*`、`/lio/localization/*`、`/map_loader/*`、`/scan_context_loop/*`、`/global_planner/*`、`/local_planner/*`);**生态/标准 topic 不加前缀**(`/livox/*`、`/cmd_vel`、`/goal_pose`、`/initialpose`、`/tf*`)。`local_planner/cmd_vel` 在 driver 节点 launch remap 回 `/cmd_vel`
- **上游代码约定**:仅 **boundary adapter** 层 patch(frame_id 字面量 `#define`、topic yaml 字段、launch remap),**不修改算法**
- **许可证**:项目对外保留各上游原许可证
- **测试入口**:**仅 bag replay**(`/media/lenovo/disk/planner_ws/data-rosbag2/Campus3` 截电梯末段);禁用 GazeboQuadbot(频率低)与实机自录 bag(CustomMsg 丢包)
- **提交规范**:Conventional Commits,每次可验证任务完成后必须 commit
- **colcon cmake 标志(强制)**:所有 `colcon build` 命令必须传 `--cmake-args -DHUMBLE_ROS=humble`,否则 `livox_ros_driver2` 会因 `LIVOX_INTERFACES_INCLUDE_DIRECTORIES=NOTFOUND` 失败
- **依赖项前置**:Livox-SDK2 必须已 source build 到 `/usr/local/`(非 apt、非 git submodule);`rosdep install` 可能报 apr-1 / libapr1-dev 缺失,但 optional 不阻塞

## 设计摘要 (2026-08-17 修订)

> 完整设计见 `docs/superpowers/specs/2026-08-17-go2w-3d-nav-design.md`(本地 spec,gitignored)。本节为缓存摘要。

**架构**:9 个组件协作,数据流为 `sensor → lio_mapping → save_map → map_loader_node → lio_localization + scan_context_loop_node → global_planner → local_planner → driver`。

**关键决策**:

| 项 | 决策 |
|---|---|
| 建图 vs 定位 | **双节点**:`lio_mapping`(建图)与 `lio_localization`(运行) |
| TF 根帧 | `map`;`map → odom` 由 lio_localization 发,带回环修正 |
| Topic 迁移 | launch remap 集中表,LIO yaml 零修改;SCAN-Planner 仅改两行 yaml |
| 定位初始位姿 | scan_context 粗定位 + KISS-ICP 精定位 |
| 地图存储 | PCD(mapping 产出)+ `map_loader_node` 离线转 OctoMap |
| 目标点 | InteractiveMarker 3D 点 + 朝向(发布 `/goal_pose`) |
| Launch | 一份 `bringup_nav3d.launch.py` + `mode:=sim/real/bag` 切换 |
| 全局规划 | 接口定,算法选型(jie_3d_nav vs OctoPlanner3D)留下一 spec |
| GPS 因子 | 保留 SC-PGO 代码路径,默认 `gps_factor_enabled:=false`,未来半开放园区启用 |
| 错误处理 | 全栈 STOP 协议(`/system/heartbeat`);`/initialpose` fallback;score < 阈值自动重定位 |

**任务分解**:14 项任务,5 个里程碑(M1 mapping 跑通 → M2 localization 跑通 → M3 端到端 → M4 异常路径 → M5 实机)。详见 spec §13。

**TF 树 / Topic 表**:见 spec §6 / §7。







## 暂拟的 Gazebo 仿真教程

### 1. 前置依赖

```bash
# ROS 2 Humble + Gazebo 11 (Classic)
sudo apt-get install -y ros-humble-gazebo-ros-pkgs ros-humble-robot-localization ros-humble-gazebo-ros2-control

# 本工作区 build
cd /home/ww/program/projects/go2w_outdoor_3d/Nav3D
source /opt/ros/humble/setup.zsh
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DHUMBLE_ROS=humble
source install/setup.zsh
```

2. 仿真底座(GazeboQuadbot)

仿真基于外部 workspace [GazeboQuadbot](`/media/lenovo/disk/Embodied_AI/GazeboQuadbot`),提供 Go2W 模型 + **Livox Mid-360 激光仿真** + IMU + 室内场景 world。

```bash
# 终端 1: 启动 Gazebo 仿真 (Go2W + mid360 + bigHHH 室内场景)
source /opt/ros/humble/setup.zsh
source /home/ww/program/projects/go2w_outdoor_3d/GazeboQuadbot/install/setup.zsh
ros2 launch go2w_config go2w_lidar_gps.launch.py
```

启动后话题:

- `/livox/lidar` — mid360 激光 (sensor_msgs/PointCloud2, PointXYZI)
- `/livox/imu` — IMU (200 Hz)
- `/cmd_vel` — 狗运动控制入口 (teleop 按键有效)

### 3. LIO 建图(Elevator-LIO)

```bash
# 终端 2: 启动 LIO mapping 模式 (消费 /livox/*, 累积 ikd-tree 地图)
cd /home/ww/program/projects/go2w_outdoor_3d/Nav3D
source install/setup.zsh
source /home/ww/program/projects/go2w_outdoor_3d/GazeboQuadbot/install/setup.zsh
ros2 launch lio start_ros2.launch.py config_path:=root_config_sim.yaml use_sim_time:=true
```

关键配置：

- `yaml/sensors/mid360_sim.yaml`:仿真专用传感器配置(外参已按 GazeboQuadbot URDF 标定)
  - `lidar_type: 2`(PointCloud2 输入,非真实 Livox CustomMsg;仿真插件发布的是标准 PointCloud2)
  - `imu_t_lidar = (-0.011, -0.02329, 0.04412)`(mid360 内置 IMU)
  - `lidar_R_body = R_y(-0.4)`, `lidar_t_body = (-0.1796, 0, -0.2171)`(对应 URDF 安装 `xyz 0.25 0 0.13, rpy 0 0.4 0`)
- `yaml/runtime/mapping_sim.yaml`:`global_map_pub_enable: true`(RViz 可视化全局地图)
- `use_sim_time:=true`:仅作用于 RViz(仿真 TF 时间域)；lio 节点流水线由消息时间戳驱动，保持墙钟节奏

### 4. 驱动狗运动 + 建图

```bash
# 终端 3: teleop 控制狗走动 (按 i=前进, j/l=转向, k=停)
source /opt/ros/humble/setup.zsh
source /home/ww/program/projects/go2w_outdoor_3d/GazeboQuadbot/install/setup.zsh
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

- **RViz 订阅** `/lio/mapping/global_map` 查看实时建图
- **RViz 订阅** `/lio_node/path_topic_name` 查看定位轨迹
- 驱动狗走遍场景所有房间/走廊,LIO 实时累积地图
