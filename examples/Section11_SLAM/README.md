# Section 11 — SLAM with SLAM Toolbox

## Overview

**SLAM** (Simultaneous Localization and Mapping) allows a robot to build a map of an unknown environment while simultaneously tracking its own position within that map — with no prior knowledge of the surroundings. This section uses **SLAM Toolbox**, the recommended SLAM solution for ROS 2 Humble, to generate 2D occupancy grid maps from LIDAR scan data.

By the end of this section the bumperbot can explore a simulated world, produce a saved map, and re-use it for localization in Section 12.

## Workspace Structure

```
bumperbot_ws/src/
├── bumperbot_description/      # Robot model with LIDAR sensor
├── bumperbot_controller/       # Differential drive controller
└── bumperbot_mapping/          # SLAM Toolbox launch files and parameter files
```

## How SLAM Works

### The Occupancy Grid

The output of SLAM is an **occupancy grid** — a 2D grid where every cell holds a probability value:

$$P(\text{cell occupied}) \in [0, 1]$$

- $P \approx 1$ → obstacle confirmed (dark cell in RViz)
- $P \approx 0$ → free space confirmed (light cell)
- $P = 0.5$ → unknown (grey cell)

The grid is published as `nav_msgs/OccupancyGrid` on the `/map` topic and visualized in RViz with the **Map** display.

### Scan Matching

SLAM Toolbox aligns successive LIDAR scans to estimate the robot's motion. Given the current scan $z_t$ and the previous scan $z_{t-1}$, it finds the rigid transform $T$ that minimizes the alignment error:

$$T^* = \arg\min_T \sum_i \| z_t^{(i)} - T \cdot z_{t-1}^{(i)} \|^2$$

This is solved iteratively using **Ceres** (a non-linear least-squares solver).

### Loop Closure

As the robot travels, small alignment errors accumulate — the estimated path drifts. When the robot revisits a known location, SLAM Toolbox detects the **loop closure**, computes a correction, and propagates it backward through the entire pose graph to eliminate drift.

```
pose graph before loop closure:
  x1 → x2 → x3 → x4 → x5 → x6
                              ↑ recognizes x1
               drift correction applied globally
```

### SLAM Toolbox Operating Modes

| Mode | Use case |
|------|----------|
| `online_async` | Real-time SLAM on a live or simulated robot (recommended) |
| `online_sync` | Real-time SLAM with blocking scan processing (more accurate, slower) |
| `offline` | Post-process a pre-recorded `.bag` file to generate a map |
| `localization` | Localize within an existing map without modifying it |
| `lifelong` | Continuously update an existing map as the environment changes over time |

## Required Packages

```bash
sudo apt install ros-humble-slam-toolbox
```

> Pre-installed in the course Docker image.

## Key Parameters (`slam_params.yaml`)

```yaml
slam_toolbox:
  ros__parameters:
    # Sensor and frame configuration
    odom_frame:        odom
    map_frame:         map
    base_frame:        base_footprint
    scan_topic:        /scan

    # Map resolution and size
    resolution:        0.05      # metres per cell
    max_laser_range:   12.0      # metres

    # Loop closure
    minimum_travel_distance:  0.5   # metres before adding a new node
    minimum_travel_heading:   0.5   # radians before adding a new node

    # Mode
    mode:              mapping    # or localization
```

## Running the Examples

```bash
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash
```

### Online SLAM in simulation

```bash
# Terminal 1 — launch Gazebo simulation
ros2 launch bumperbot_description gazebo.launch.py

# Terminal 2 — launch SLAM Toolbox (online async)
ros2 launch bumperbot_mapping slam.launch.py

# Terminal 3 — drive the robot to build the map
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/bumperbot_controller/cmd_vel

# Terminal 4 — visualize in RViz
rviz2
# Add display: Map → topic /map
# Add display: LaserScan → topic /scan
# Add display: TF
```

### Save the map when done

```bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/my_map
# Produces: my_map.pgm (image) + my_map.yaml (metadata)
```

### Localize on a saved map (no new mapping)

```bash
ros2 launch bumperbot_mapping localization.launch.py \
  map:=$HOME/ros2_ws/my_map.yaml
```

## The Map Files

After saving, two files are produced:

| File | Contents |
|------|----------|
| `my_map.pgm` | Greyscale image of the occupancy grid (white=free, black=occupied, grey=unknown) |
| `my_map.yaml` | Metadata: resolution, origin, thresholds for free/occupied pixels |

Example `my_map.yaml`:
```yaml
image: my_map.pgm
resolution: 0.050000
origin: [-10.0, -10.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

## TF Tree During SLAM

```
map
 └── odom          (published by SLAM Toolbox)
      └── base_footprint   (published by odometry controller)
           └── base_link
                └── laser_frame
```

SLAM Toolbox publishes the `map → odom` transform. This is the correction that accounts for drift that pure odometry cannot detect.

## Dependencies

| Package | Purpose |
|---------|---------|
| `slam_toolbox` | 2D SLAM with pose graph optimization and loop closure |
| `nav2_map_server` | `map_saver_cli` tool for saving maps to disk |
| `sensor_msgs` | `LaserScan` message type |
| `nav_msgs` | `OccupancyGrid` message type |

## What You Will Learn

- How occupancy grid maps represent the environment probabilistically
- How scan matching estimates robot motion from LIDAR data
- How loop closure corrects accumulated odometry drift
- How to configure and launch SLAM Toolbox for real-time mapping
- How to save, load, and switch between mapping and localization modes
- The role of the `map → odom` transform in the ROS 2 navigation TF tree
