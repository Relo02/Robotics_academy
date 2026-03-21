# Section 4 — Forward Kinematics

## Overview

This section introduces the mathematical foundations of **robot kinematics** — computing the position and orientation of robot parts relative to each other. You will implement coordinate frame transformations manually using rotation matrices and translation vectors, using the turtlesim simulator as a visual test environment.

## Workspace Structure

```
bumperbot_ws/src/
├── bumperbot_description/      # Robot model
├── bumperbot_controller/       # Controller stack
├── bumperbot_cpp_examples/     # C++ kinematics node
└── bumperbot_py_examples/      # Python kinematics node
```

## Key Nodes

### `simple_turtlesim_kinematics` (Python & C++)

Subscribes to the poses of two turtles in turtlesim (`/turtle1/pose`, `/turtle2/pose`) and computes the **relative transformation** between them.

For each pair of poses the node computes and logs:

| Output | Description |
|--------|-------------|
| Translation vector `[Tx, Ty]` | Position of turtle2 relative to turtle1 in turtle1's frame |
| Rotation matrix `R` | 2×2 matrix encoding the relative heading angle |
| Full transformation | Combined translation + rotation describing the rigid body transform |

The rotation matrix is computed as:

```
R = | cos(Δθ)  -sin(Δθ) |
    | sin(Δθ)   cos(Δθ) |
```

where `Δθ = θ2 - θ1`.

## Key Concepts

- **Coordinate frame**: a reference system defined by an origin and orientation
- **Homogeneous transform**: a 4×4 matrix combining rotation and translation
- **Rotation matrix**: a 3×3 (or 2×2 in 2D) orthonormal matrix encoding orientation
- **Forward kinematics**: computing end-effector pose from known joint states

## Running the Examples

```bash
# Terminal 1 — start turtlesim
ros2 run turtlesim turtlesim_node

# Terminal 2 — spawn a second turtle
ros2 service call /spawn turtlesim/srv/Spawn "{x: 4.0, y: 4.0, theta: 0.0, name: 'turtle2'}"

# Terminal 3 — run the kinematics node
ros2 run bumperbot_py_examples simple_turtlesim_kinematics

# Terminal 4 — move turtle1 to trigger updates
ros2 run turtlesim turtle_teleop_key
```

## Dependencies

| Package | Purpose |
|---------|---------|
| `turtlesim` | 2D turtle simulator for prototyping |
| `geometry_msgs` | `Pose`, `Point`, `Quaternion` message types |
| `rclpy` / `rclcpp` | ROS 2 client libraries |

## What You Will Learn

- How to represent robot poses mathematically
- How to compute 2D and 3D rigid body transformations
- How rotation matrices relate to heading angles
- How to subscribe to pose data and process it in a ROS 2 node
- Why coordinate frames are critical for multi-body systems
