"""
spawn.launch.py
---------------
Launches Gazebo with the diff-drive robot and RViz for visualization.

Equivalent terminal commands (run each in a separate terminal):

  Terminal 1 — start Gazebo (empty world):
    ros2 launch gazebo_ros gazebo.launch.py

  Terminal 2 — publish robot description and TF tree:
    ros2 run robot_state_publisher robot_state_publisher \
        --ros-args -p robot_description:="$(xacro /path/to/diff_drive.urdf.xacro)"

  Terminal 3 — spawn the robot model in the running Gazebo instance:
    ros2 run gazebo_ros spawn_entity.py \
        -topic robot_description -entity diff_drive_robot \
        -x 0.0 -y 0.0 -z 0.1

  Terminal 4 — load and activate the joint-state broadcaster:
    ros2 control load_controller --set-state active joint_state_broadcaster

  Terminal 5 — load and activate the differential-drive controller:
    ros2 control load_controller --set-state active diff_drive_controller

  Terminal 6 — open RViz with the pre-saved SLAM configuration:
    ros2 run rviz2 rviz2 -d /path/to/slam.rviz
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                             IncludeLaunchDescription, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg       = get_package_share_directory('diff_drive_description')
    gazebo_pkg = get_package_share_directory('gazebo_ros')

    # ------------------------------------------------------------------ args
    # ros2 launch diff_drive_description spawn.launch.py use_sim_time:=true
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # ros2 launch diff_drive_description spawn.launch.py world:=/path/to/world.world
    world_file   = LaunchConfiguration('world', default='')

    # ------------------------------------------------------------------ URDF
    # Equivalent CLI:
    #   xacro /path/to/diff_drive.urdf.xacro
    # The result string is passed directly to robot_state_publisher.
    xacro_file = os.path.join(pkg, 'urdf', 'diff_drive.urdf.xacro')
    robot_description = Command(['xacro ', xacro_file])

    # ------------------------------------------------------------------ robot_state_publisher
    # Equivalent CLI (Terminal 2):
    #   ros2 run robot_state_publisher robot_state_publisher \
    #       --ros-args -p robot_description:="$(xacro /path/to/diff_drive.urdf.xacro)" \
    #                  -p use_sim_time:=true
    # Publishes:
    #   /robot_description  (std_msgs/String)
    #   /tf and /tf_static  (all fixed and moving frames from the URDF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    # ------------------------------------------------------------------ Gazebo simulator
    # Equivalent CLI (Terminal 1):
    #   ros2 launch gazebo_ros gazebo.launch.py world:=/path/to/world.world
    # Starts the Gazebo server (gzserver) + client (gzclient) GUI.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items(),
    )

    # ------------------------------------------------------------------ spawn robot in Gazebo
    # Equivalent CLI (Terminal 3) — run AFTER Gazebo is up:
    #   ros2 run gazebo_ros spawn_entity.py \
    #       -topic robot_description \
    #       -entity diff_drive_robot \
    #       -x 0.0 -y 0.0 -z 0.1
    # Reads /robot_description topic and inserts the model at the given pose.
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'diff_drive_robot',
            '-x', '0.0', '-y', '0.0', '-z', '0.1',
        ],
        output='screen',
    )

    # ------------------------------------------------------------------ joint state broadcaster
    # Equivalent CLI (Terminal 4) — run AFTER the robot is spawned:
    #   ros2 control load_controller --set-state active joint_state_broadcaster
    # Broadcasts /joint_states so robot_state_publisher can compute forward kinematics.
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen',
    )

    # ------------------------------------------------------------------ diff-drive controller
    # Equivalent CLI (Terminal 5) — run AFTER joint_state_broadcaster is active:
    #   ros2 control load_controller --set-state active diff_drive_controller
    # Subscribes to /cmd_vel (geometry_msgs/Twist) and publishes /odom.
    # Send a velocity command manually with:
    #   ros2 topic pub /cmd_vel geometry_msgs/Twist \
    #       "{linear: {x: 0.2}, angular: {z: 0.1}}" --once
    load_diff_drive_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'diff_drive_controller'],
        output='screen',
    )

    # Chain: spawn → joint_state_broadcaster → diff_drive_controller
    start_jsb_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[load_joint_state_broadcaster],
        )
    )
    start_ddc_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_diff_drive_controller],
        )
    )

    # ------------------------------------------------------------------ RViz
    # Equivalent CLI (Terminal 6):
    #   ros2 run rviz2 rviz2 -d /path/to/slam.rviz
    # Visualizes: robot model, LiDAR scan, occupancy grid, TF tree, EKF odometry.
    rviz_config = os.path.join(
        get_package_share_directory('slam_bringup'), 'rviz', 'slam.rviz'
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use Gazebo simulation clock'),
        DeclareLaunchArgument('world', default_value='',
                              description='Full path to Gazebo world file'),
        robot_state_publisher,   # Terminal 2 equivalent
        gazebo,                  # Terminal 1 equivalent
        spawn_robot,             # Terminal 3 equivalent
        start_jsb_after_spawn,   # Terminal 4 equivalent (triggered by spawn exit)
        start_ddc_after_jsb,     # Terminal 5 equivalent (triggered by JSB exit)
        rviz,                    # Terminal 6 equivalent
    ])
