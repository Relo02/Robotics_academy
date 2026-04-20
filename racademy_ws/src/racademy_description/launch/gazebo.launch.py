#!/usr/bin/env python3
"""Launch Gazebo Sim with the racademy robot description."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("racademy_description")
    share_root = str(Path(pkg_share).parent)

    model = LaunchConfiguration("model")
    world = LaunchConfiguration("world")
    entity_name = LaunchConfiguration("entity_name")
    use_sim_time = LaunchConfiguration("use_sim_time")
    is_ignition = LaunchConfiguration("is_ignition")

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(pkg_share, "urdf", "racademy.urdf.xacro"),
        description="Absolute path to the robot Xacro file",
    )
    world_arg = DeclareLaunchArgument(
        name="world",
        default_value="empty.sdf",
        description="Gazebo world file passed to gz_sim.launch.py",
    )
    entity_name_arg = DeclareLaunchArgument(
        name="entity_name",
        default_value="racademy",
        description="Entity name used when spawning the robot in Gazebo",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="true",
        description="Use Gazebo simulation time",
    )
    is_ignition_arg = DeclareLaunchArgument(
        name="is_ignition",
        default_value="false",
        description="Use the legacy ign_ros2_control plugin instead of gz_ros2_control",
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
        model_arg,
        world_arg,
        entity_name_arg,
        use_sim_time_arg,
        is_ignition_arg,
        ign_resource,
        gz_resource,
        rsp_node,
        gazebo,
        TimerAction(period=2.0, actions=[spawn_entity]),
        clock_bridge,
    ])
