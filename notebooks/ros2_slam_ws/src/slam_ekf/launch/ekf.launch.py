"""
ekf.launch.py
-------------
Launches the EKF state-estimation node.

Equivalent terminal command (no launch file):
  ros2 run slam_ekf ekf_node \\
      --ros-args \\
      -p process_noise_x:=0.01 \\
      -p process_noise_y:=0.01 \\
      -p process_noise_theta:=0.01 \\
      -p process_noise_v:=0.05 \\
      -p process_noise_omega:=0.05 \\
      -p measurement_noise_v:=0.1 \\
      -p measurement_noise_omega:=0.1 \\
      -p publish_tf:=true \\
      -p use_sim_time:=true

To inspect the filtered pose:
  ros2 topic echo /odometry/filtered

To inspect the TF tree:
  ros2 run tf2_tools view_frames
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # EKF node
    # Subscribes to:
    #   /odom          (nav_msgs/Odometry)   — wheel encoder odometry
    #   /imu/data      (sensor_msgs/Imu)     — IMU angular velocity
    # Publishes:
    #   /odometry/filtered  (nav_msgs/Odometry)   — EKF posterior
    # Broadcasts TF:
    #   odom → base_footprint
    ekf_node = Node(
        package='slam_ekf',
        executable='ekf_node',
        name='ekf_localization',
        parameters=[{
            'use_sim_time':           use_sim_time,
            # ---- Process noise (Q diagonal) -------------------------
            # Higher value = trust prediction less, react faster to measurements
            'process_noise_x':        0.01,
            'process_noise_y':        0.01,
            'process_noise_theta':    0.01,
            'process_noise_v':        0.05,
            'process_noise_omega':    0.05,
            # ---- Measurement noise (R diagonal) ---------------------
            # Higher value = trust wheel odometry less
            'measurement_noise_v':    0.1,
            'measurement_noise_omega': 0.1,
            # ---- Broadcast odom->base_footprint TF? -----------------
            'publish_tf': True,
        }],
        output='screen',
        remappings=[
            # If your odom topic has a different name, remap here:
            # ('odom', '/diff_drive_controller/odom'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use /clock from Gazebo'),
        ekf_node,
    ])
