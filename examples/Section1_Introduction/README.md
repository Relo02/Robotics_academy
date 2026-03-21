# Section 1 — Introduction to ROS 2

## Overview

This section introduces the foundational concepts of ROS 2 (Robot Operating System 2). You will create your first nodes, publish and subscribe to messages, and understand the basic communication architecture that underpins every ROS 2 application.

## Workspace Structure

```
bumperbot_ws/src/
├── bumperbot_cpp_examples/     # C++ implementation of basic nodes
└── bumperbot_py_examples/      # Python implementation of basic nodes
```

## Packages

### `bumperbot_cpp_examples`
C++ nodes demonstrating the core ROS 2 communication primitives.

| Node | File | Description |
|------|------|-------------|
| `simple_publisher` | `src/simple_publisher.cpp` | Publishes a `std_msgs/String` message to `/chatter` at 1 Hz |
| `simple_subscriber` | `src/simple_subscriber.cpp` | Subscribes to `/chatter` and logs received messages |

### `bumperbot_py_examples`
Python equivalents of the same nodes, using `rclpy`.

| Node | File | Description |
|------|------|-------------|
| `simple_publisher` | `bumperbot_py_examples/simple_publisher.py` | Publishes a `std_msgs/String` to `/chatter` at 1 Hz |
| `simple_subscriber` | `bumperbot_py_examples/simple_subscriber.py` | Subscribes to `/chatter` and logs received messages |

## Key ROS 2 Concepts

- **Node**: the basic unit of computation in ROS 2 — a process that performs a specific task
- **Publisher**: sends messages on a named topic
- **Subscriber**: receives messages from a named topic
- **Timer**: triggers a callback at a fixed rate
- **Logger**: structured logging via `get_logger().info(...)`

## Running the Examples

Build the workspace first:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Python publisher + subscriber

```bash
# Terminal 1
ros2 run bumperbot_py_examples simple_publisher

# Terminal 2
ros2 run bumperbot_py_examples simple_subscriber
```

### C++ publisher + subscriber

```bash
# Terminal 1
ros2 run bumperbot_cpp_examples simple_publisher

# Terminal 2
ros2 run bumperbot_cpp_examples simple_subscriber
```

### Inspect the topic

```bash
ros2 topic list
ros2 topic echo /chatter
ros2 topic hz /chatter
```

## Dependencies

| Package | Purpose |
|---------|---------|
| `rclcpp` | C++ ROS 2 client library |
| `rclpy` | Python ROS 2 client library |
| `std_msgs` | Standard message types (`String`, `Int32`, etc.) |

## What You Will Learn

- How to create a ROS 2 node in both Python and C++
- The publisher/subscriber pattern (asynchronous, decoupled communication)
- How to use timers for periodic callbacks
- How to inspect topics from the command line with `ros2 topic`