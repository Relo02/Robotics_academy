# Section 6 — The TF2 Library

## Overview

This section provides a comprehensive introduction to **TF2** — ROS 2's transform library. TF2 maintains a tree of coordinate frames and allows any node to query the transformation between any two frames at any point in time. It is essential for multi-body robots, sensor fusion, and navigation.

## Workspace Structure

```
bumperbot_ws/src/
├── bumperbot_msgs/             # Custom service: GetTransform
├── bumperbot_description/      # Robot model
├── bumperbot_controller/       # Controller stack
├── bumperbot_cpp_examples/     # C++ TF2 broadcaster + listener
└── bumperbot_py_examples/      # Python TF2 broadcaster + listener
```

## Custom Interfaces — `bumperbot_msgs`

| Service | Request | Response |
|---------|---------|----------|
| `GetTransform` | `frame_id` (string), `child_frame_id` (string) | `TransformStamped`, `success` (bool) |

## Key Node — `simple_tf_kinematics`

This is the central node of this section, available in both Python and C++. It demonstrates all major TF2 concepts in one place.

### What it does

| Feature | API Used | Description |
|---------|----------|-------------|
| Static transform | `StaticTransformBroadcaster` | Publishes a fixed `base_link → top_link` transform once at startup |
| Dynamic transform | `TransformBroadcaster` | Publishes an animated `odom → base_footprint` transform at 10 Hz |
| Transform query | `TransformListener` + `Buffer` | Looks up any transform via the TF2 buffer |
| Service server | `create_service` | Exposes `/get_transform` service using the custom `GetTransform` interface |
| Quaternion math | `tf_transformations` | Converts Euler angles to quaternions, multiplies and inverts quaternions |

### Transform Tree Produced

```
world (odom)
    └── base_footprint     [dynamic — moves over time]
            └── base_link
                    └── top_link   [static — fixed offset]
```

## Running the Examples

```bash
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash

# Terminal 1 — run the TF2 kinematics node
ros2 run bumperbot_py_examples simple_tf_kinematics

# Inspect the transform tree
ros2 run tf2_tools view_frames   # saves frames.pdf
ros2 run tf2_ros tf2_echo odom base_footprint

# Call the custom GetTransform service
ros2 service call /get_transform bumperbot_msgs/srv/GetTransform \
  "{ frame_id: 'odom', child_frame_id: 'base_footprint' }"
```

### Full simulation with TF2

```bash
ros2 launch bumperbot_description gazebo.launch.py
ros2 launch bumperbot_controller controller.launch.py
```

Then open RViz and add the **TF** display to visualize all active frames.

## TF2 Key Concepts

| Concept | Description |
|---------|-------------|
| **Frame** | A named coordinate system (e.g., `odom`, `base_link`, `camera_link`) |
| **Transform tree** | A directed acyclic graph of frame relationships |
| **Static broadcaster** | For fixed transforms (e.g., sensor mounted on chassis) |
| **Dynamic broadcaster** | For time-varying transforms (e.g., robot pose in world) |
| **Buffer + Listener** | Cache of historical transforms; enables time-travel lookups |
| **Quaternion** | 4D representation of 3D rotation; avoids gimbal lock |

## Dependencies

| Package | Purpose |
|---------|---------|
| `tf2` | Core TF2 library |
| `tf2_ros` | ROS 2 bindings for TF2 (broadcaster, listener, buffer) |
| `tf_transformations` | Quaternion utilities (`quaternion_from_euler`, `quaternion_multiply`) |
| `tf2_tools` | CLI tools (`view_frames`, `tf2_echo`) |
| `geometry_msgs` | `TransformStamped`, `Quaternion`, `Vector3` |

## What You Will Learn

- How TF2 manages coordinate frames across a distributed ROS 2 system
- The difference between static and dynamic transform broadcasters
- How to query past and present transforms using `TransformListener` and `Buffer`
- Quaternion representation and common operations (euler→quaternion, multiply, inverse)
- How to expose transform data as a ROS 2 service
- How to visualize the transform tree with `view_frames` and RViz
