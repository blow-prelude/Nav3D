#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


MAPPING_REMAPS = [
    ("/LIO/set_elevator_flag", "/lio/mapping/set_elevator_flag"),
    ("/LIO/odom_imu", "/lio/mapping/odom_imu"),
    ("/LIO/odom_vehicle", "/lio/mapping/odom_body"),
    ("/LIO/clouds_lidar", "/lio/mapping/clouds_lidar"),
    ("/LIO/clouds_lidar/effect", "/lio/mapping/clouds_lidar/effect"),
    ("/LIO/clouds_lidar/reject", "/lio/mapping/clouds_lidar/reject"),
    ("/LIO/global_map", "/lio/mapping/global_map"),
    ("/LIO/ikdtree", "/lio/mapping/ikdtree"),
    ("/LIO/in_elevator", "/lio/mapping/in_elevator"),
    ("/LIO/elevator_state", "/lio/mapping/elevator_state"),
]


def generate_launch_description():
    package_share = FindPackageShare("lio")
    config_path = LaunchConfiguration("config_path")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument("config_path", default_value="root_config.yaml"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        # 仅作用于 rviz（显示时钟）。lio 节点保持墙钟节奏：
        # 其流水线完全由消息时间戳驱动，定时器只负责推进 FSM；
        # 若也绑到 /clock（Gazebo 常见只有 ~10Hz），FSM 消费 IMU 的速率会被
        # 压到时钟频率量级，建图掉到 ~1Hz 且永远追不上仿真时间。
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="lio",
            executable="lio",
            name="lio_node",
            output="screen",
            parameters=[{"config_path": config_path}],
            remappings=MAPPING_REMAPS,
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", PathJoinSubstitution([package_share, "rviz", "LIO_ros2.rviz"])],
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(use_rviz),
        ),
    ])
