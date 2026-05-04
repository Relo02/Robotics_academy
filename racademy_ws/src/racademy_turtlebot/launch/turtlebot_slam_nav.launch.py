#!/usr/bin/env python3
"""
LiDAR SLAM + Nav2 pipeline for the turtlebot (Gazebo Harmonic).

Equivalent to the tutorial at:
  https://roboticsbackend.com/ros2-nav2-generate-a-map-with-slam_toolbox/

Pipeline:
  Gazebo (slam_world.sdf with sensors plugin)
    └── /scan  ──► slam_toolbox (online async) ──► map → odom TF + /map
    └── /odom  ──► Nav2 (navigation_launch.py) ──► /cmd_vel
                         │
                        RViz2 (nav2_default_view)

Drive with:
  ros2 run teleop_twist_keyboard teleop_twist_keyboard

Save map:
  ros2 run nav2_map_server map_saver_cli -f ~/racademy_ws/maps/tb_map
  ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
    "{filename: ~/racademy_ws/maps/tb_graph}"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_params_file = LaunchConfiguration("slam_params_file")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    world = LaunchConfiguration("world")

    tb_pkg = get_package_share_directory("racademy_turtlebot")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tb_pkg, "launch", "gazebo.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "world": world,
        }.items(),
    )

    broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    diff_drive = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Bridge LiDAR from Gazebo transport → ROS 2.
    # Topic is absolute "/scan" in the sensor SDF, matching the bridge argument.
    scan_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=["/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"],
    )

    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_params_file, {"use_sim_time": use_sim_time}],
    )

    # Nav2 navigation stack — path planning and local control
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("nav2_bringup"), "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": nav2_params_file,
        }.items(),
    )

    # Relay Nav2's /cmd_vel → /diff_drive_controller/cmd_vel_unstamped
    cmd_vel_relay = Node(
        package="racademy_turtlebot",
        executable="cmd_vel_relay",
        name="cmd_vel_relay",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            os.path.join(get_package_share_directory("nav2_bringup"), "rviz", "nav2_default_view.rviz"),
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "world",
            default_value=os.path.join(tb_pkg, "worlds", "slam_office.sdf"),
        ),
        DeclareLaunchArgument(
            "slam_params_file",
            default_value=os.path.join(tb_pkg, "config", "slam_toolbox_turtlebot.yaml"),
        ),
        DeclareLaunchArgument(
            "nav2_params_file",
            default_value=os.path.join(tb_pkg, "config", "nav2_params.yaml"),
        ),
        gazebo,
        # Give Gazebo 3 s to fully load before spawning controllers and bridge
        TimerAction(period=3.0, actions=[broadcaster, diff_drive, scan_bridge]),
        slam_toolbox,
        nav2,
        cmd_vel_relay,
        rviz,
    ])
