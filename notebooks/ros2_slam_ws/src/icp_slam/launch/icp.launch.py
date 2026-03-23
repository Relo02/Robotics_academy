"""
icp.launch.py
-------------
Launches the ICP scan-matching SLAM node.

Equivalent terminal command (no launch file):
  ros2 run icp_slam icp_node \\
      --ros-args \\
      -p max_iter:=50 \\
      -p tolerance:=0.001 \\
      -p min_scan_dist:=0.5 \\
      -p map_resolution:=0.05 \\
      -p map_size:=200 \\
      -p use_sim_time:=true

Useful monitoring commands:
  # Watch the live map header (avoid printing all cell data)
  ros2 topic echo /map --no-arr

  # Watch the ICP-derived robot pose in the map frame
  ros2 topic echo /icp_pose

  # Check the full TF chain from map to lidar
  ros2 run tf2_ros tf2_echo map lidar_link

  # Visualise in RViz (add Map and TF displays)
  ros2 run rviz2 rviz2
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # ICP SLAM node
    # Subscribes to:
    #   /scan           (sensor_msgs/LaserScan)   — 2-D LiDAR data
    # Publishes:
    #   /map            (nav_msgs/OccupancyGrid)  — incremental occupancy map
    #   /icp_pose       (geometry_msgs/PoseStamped) — ICP-derived global pose
    # Broadcasts TF:
    #   map → odom
    icp_node = Node(
        package='icp_slam',
        executable='icp_node',
        name='icp_slam',
        parameters=[{
            'use_sim_time':    use_sim_time,
            # Max ICP iterations per scan pair
            'max_iter':        50,
            # Convergence threshold [m] and [rad]
            'tolerance':       0.001,
            # Minimum robot displacement [m] before updating the reference scan
            # Lower = more frequent ICP calls (more CPU); higher = faster drift
            'min_scan_dist':   0.5,
            # Occupancy grid resolution [m/cell]
            'map_resolution':  0.05,
            # Map size [cells per side]; total map = map_size × map_size cells
            # At 0.05 m/cell, 200×200 = 10 m × 10 m
            'map_size':        200,
        }],
        output='screen',
        remappings=[
            # Remap if your LiDAR publishes on a different topic:
            # ('scan', '/lidar/scan'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use /clock from Gazebo'),
        icp_node,
    ])
