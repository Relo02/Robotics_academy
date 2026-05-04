#!/usr/bin/env python3
"""Bring up racademy with SLAM localization and Nav2 navigation stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")
    map_file_name = LaunchConfiguration("map_file_name")
    slam_params_file = LaunchConfiguration("slam_params_file")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    autostart = LaunchConfiguration("autostart")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")

    nav2_pkg = get_package_share_directory("racademy_nav2")
    slam_pkg = get_package_share_directory("racademy_slam")
    nav2_bringup_pkg = get_package_share_directory("nav2_bringup")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock",
    )

    world_arg = DeclareLaunchArgument(
        "world",
        default_value="empty.sdf",
        description="Gazebo world file",
    )

    map_file_name_arg = DeclareLaunchArgument(
        "map_file_name",
        default_value="/home/ros2user/ros2_ws/maps/racademy_graph",
        description="Serialized SLAM pose-graph base path",
    )

    slam_params_file_arg = DeclareLaunchArgument(
        "slam_params_file",
        default_value=os.path.join(slam_pkg, "config", "slam_toolbox_localization.yaml"),
        description="SLAM Toolbox localization params",
    )

    nav2_params_file_arg = DeclareLaunchArgument(
        "nav2_params_file",
        default_value=os.path.join(nav2_pkg, "config", "nav2_params.yaml"),
        description="Nav2 parameters file",
    )

    autostart_arg = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup nav2 stack",
    )

    use_composition_arg = DeclareLaunchArgument(
        "use_composition",
        default_value="False",
        description="Use composition for nav2 bringup",
    )

    use_respawn_arg = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Respawn nav2 nodes on failure",
    )

    slam_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_pkg, "launch", "slam_localization.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "world": world,
            "map_file_name": map_file_name,
            "slam_params_file": slam_params_file,
        }.items(),
    )

    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_pkg, "launch", "navigation_launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": nav2_params_file,
            "autostart": autostart,
            "use_composition": use_composition,
            "use_respawn": use_respawn,
        }.items(),
    )

    cmd_vel_relay = Node(
        package="racademy_nav2",
        executable="cmd_vel_relay.py",
        name="cmd_vel_relay",
        output="screen",
        parameters=[
            {
                "input_topic": "/cmd_vel",
                "output_topic": "/diff_drive_controller/cmd_vel_unstamped",
            }
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        world_arg,
        map_file_name_arg,
        slam_params_file_arg,
        nav2_params_file_arg,
        autostart_arg,
        use_composition_arg,
        use_respawn_arg,
        slam_localization,
        nav2_navigation,
        cmd_vel_relay,
    ])
