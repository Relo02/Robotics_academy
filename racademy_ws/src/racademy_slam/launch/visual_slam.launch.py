#!/usr/bin/env python3
"""Visual SLAM for the duckibot using rtab-map (monocular camera + wheel odometry)."""

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
    rtabmap_params_file = LaunchConfiguration("rtabmap_params_file")

    slam_pkg = get_package_share_directory("racademy_slam")
    description_pkg = get_package_share_directory("racademy_description")
    controller_pkg = get_package_share_directory("racademy_controller")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_pkg, "launch", "gazebo.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time, "world": world}.items(),
    )

    controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_pkg, "launch", "controller.launch.py")
        ),
    )

    # Bridge monocular camera topics from Gazebo to ROS 2.
    # Gazebo camera sensor publishes on topic "camera"; camera_info on "camera/camera_info".
    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "/camera@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
    )

    rtabmap = Node(
        package="rtabmap_ros",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        parameters=[
            rtabmap_params_file,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("rgb/image", "/camera"),
            ("rgb/camera_info", "/camera/camera_info"),
            # Wheel odometry from diff_drive_controller
            ("odom", "/diff_drive_controller/odom"),
        ],
        arguments=["--delete_db_on_start"],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(slam_pkg, "rviz", "visual_slam.rviz")],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time", default_value="true", description="Use simulation clock"
        ),
        DeclareLaunchArgument(
            "world", default_value="empty.sdf", description="Gazebo world file"
        ),
        DeclareLaunchArgument(
            "rtabmap_params_file",
            default_value=os.path.join(slam_pkg, "config", "rtabmap_monocular.yaml"),
            description="Path to rtab-map parameter file",
        ),
        gazebo,
        controllers,
        camera_bridge,
        rtabmap,
        rviz,
    ])
