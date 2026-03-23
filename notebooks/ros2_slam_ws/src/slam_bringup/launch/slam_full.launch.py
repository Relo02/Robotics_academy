"""
slam_full.launch.py
--------------------
Master bringup: starts the complete SLAM stack in one command.

  ros2 launch slam_bringup slam_full.launch.py

What it does (in order):
  1.  Gazebo simulator          — physics engine + GUI
  2.  robot_state_publisher     — URDF → /robot_description + TF static frames
  3.  spawn_entity              — inserts the robot model into Gazebo
  4.  joint_state_broadcaster   — broadcasts /joint_states from ros2_control
  5.  diff_drive_controller     — subscribes /cmd_vel, publishes /odom
  6.  EKF node                  — fuses /odom + /imu/data → /odometry/filtered
                                    broadcasts odom → base_footprint TF
  7.  ICP SLAM node             — matches /scan, builds /map,
                                    broadcasts map → odom TF
  8.  RViz                      — visualisation with pre-saved config

Equivalent terminal commands (7 terminals):
  T1:  ros2 launch gazebo_ros gazebo.launch.py world:=/path/to/world.world
  T2:  ros2 run robot_state_publisher robot_state_publisher \\
           --ros-args -p robot_description:="$(xacro /path/to/diff_drive.urdf.xacro)"
  T3:  ros2 run gazebo_ros spawn_entity.py \\
           -topic robot_description -entity diff_drive_robot -x 0 -y 0 -z 0.1
  T4:  ros2 control load_controller --set-state active joint_state_broadcaster
  T5:  ros2 control load_controller --set-state active diff_drive_controller
  T6:  ros2 launch slam_ekf ekf.launch.py
  T7:  ros2 launch icp_slam icp.launch.py
  T8:  ros2 run rviz2 rviz2 -d /path/to/slam.rviz

Drive the robot manually once everything is up:
  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \\
      "{linear: {x: 0.2}, angular: {z: 0.3}}" -r 10
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
    desc_pkg   = get_package_share_directory('diff_drive_description')
    bringup_pkg = get_package_share_directory('slam_bringup')
    gazebo_pkg  = get_package_share_directory('gazebo_ros')
    ekf_pkg     = get_package_share_directory('slam_ekf')
    icp_pkg     = get_package_share_directory('icp_slam')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file   = LaunchConfiguration('world', default='')

    # ---- URDF -----------------------------------------------------------
    xacro_file        = os.path.join(desc_pkg, 'urdf', 'diff_drive.urdf.xacro')
    robot_description = Command(['xacro ', xacro_file])

    # ---- 1. robot_state_publisher ---------------------------------------
    # T2 equivalent: publishes /robot_description and static TF frames
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time':      use_sim_time,
        }],
    )

    # ---- 2. Gazebo ------------------------------------------------------
    # T1 equivalent
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items(),
    )

    # ---- 3. Spawn robot -------------------------------------------------
    # T3 equivalent
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

    # ---- 4. Joint-state broadcaster ------------------------------------
    # T4 equivalent; activated after the robot is spawned
    load_jsb = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen',
    )

    # ---- 5. Diff-drive controller ---------------------------------------
    # T5 equivalent; activated after joint_state_broadcaster is running
    # Publishes /odom and accepts /cmd_vel
    load_ddc = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active', 'diff_drive_controller'],
        output='screen',
    )

    # Chain spawn → JSB → DDC
    start_jsb_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[load_jsb],
        )
    )
    start_ddc_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_jsb,
            on_exit=[load_ddc],
        )
    )

    # ---- 6. EKF state estimation ----------------------------------------
    # T6 equivalent
    # Subscribes: /odom, /imu/data
    # Publishes:  /odometry/filtered
    # Broadcasts TF: odom → base_footprint
    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ekf_pkg, 'launch', 'ekf.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # ---- 7. ICP SLAM ----------------------------------------------------
    # T7 equivalent
    # Subscribes: /scan
    # Publishes:  /map, /icp_pose
    # Broadcasts TF: map → odom
    icp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(icp_pkg, 'launch', 'icp.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # ---- 8. RViz --------------------------------------------------------
    # T8 equivalent
    rviz_config = os.path.join(bringup_pkg, 'rviz', 'slam.rviz')
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
                              description='Path to Gazebo world file'),
        # Execution order
        robot_state_publisher,   # must be first (supplies robot_description topic)
        gazebo,
        spawn_robot,
        start_jsb_after_spawn,
        start_ddc_after_jsb,
        ekf,
        icp,
        rviz,
    ])
