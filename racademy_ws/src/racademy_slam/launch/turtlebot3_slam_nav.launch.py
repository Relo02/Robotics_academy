#!/usr/bin/env python3
"""
Convenience re-export: delegates to racademy_turtlebot turtlebot_slam_nav.launch.py.

Note: ros-humble-turtlebot3-gazebo cannot be installed on this Docker image because
it requires Gazebo Classic (gazebo 11.x), which conflicts with Gazebo Harmonic
(gz-tools2) already present. The racademy_turtlebot package provides an equivalent
self-contained robot + world that works with Gazebo Harmonic.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("racademy_turtlebot"),
                    "launch",
                    "turtlebot_slam_nav.launch.py",
                )
            )
        )
    ])
