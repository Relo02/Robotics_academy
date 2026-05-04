#!/usr/bin/env python3
"""Launch Gazebo Harmonic with the turtlebot inside slam_world.sdf.

slam_world.sdf explicitly loads gz-sim-sensors-system, which is required
for the LiDAR sensor to publish data on Gazebo transport.
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("racademy_turtlebot")
    share_root = str(Path(pkg_share).parent)

    model = LaunchConfiguration("model")
    world = LaunchConfiguration("world")
    entity_name = LaunchConfiguration("entity_name")
    use_sim_time = LaunchConfiguration("use_sim_time")
    is_ignition = LaunchConfiguration("is_ignition")

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(pkg_share, "urdf", "turtlebot.urdf.xacro"),
    )
    world_arg = DeclareLaunchArgument(
        name="world",
        # Use our custom world that has gz-sim-sensors-system loaded
        default_value=os.path.join(pkg_share, "worlds", "slam_world.sdf"),
    )
    entity_name_arg = DeclareLaunchArgument(
        name="entity_name", default_value="turtlebot",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time", default_value="true",
    )
    is_ignition_arg = DeclareLaunchArgument(
        name="is_ignition", default_value="false",
    )

    ign_resource = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=[share_root, ":", EnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", default_value="")],
    )
    gz_resource = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[share_root, ":", EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value="")],
    )

    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", model, " is_ignition:=", is_ignition]),
        value_type=str,
    )

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time}],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": ["-r -v 4 ", world]}.items(),
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", entity_name],
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )

    return LaunchDescription([
        model_arg, world_arg, entity_name_arg, use_sim_time_arg, is_ignition_arg,
        ign_resource, gz_resource,
        rsp_node,
        gazebo,
        TimerAction(period=2.0, actions=[spawn_entity]),
        clock_bridge,
    ])
