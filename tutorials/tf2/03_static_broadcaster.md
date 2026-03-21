# Tutorial 3 — Writing a Static TF2 Broadcaster

> Reference: https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Static-Broadcaster-Py.html

## What is a Static Broadcaster?

A **static broadcaster** publishes a transform that never changes over time — for example, a camera rigidly mounted on the robot chassis. Unlike a regular `TransformBroadcaster`, it publishes to the `/tf_static` topic, which is **latched**: new subscribers immediately receive the last published message. The transform is published **once** at startup.

Use a static broadcaster when:
- The transform is physically fixed (sensor bolted to the frame)
- The transform is determined at configuration time (known from CAD/calibration)
- You never need to update it at runtime

## When to Use Static vs. Dynamic

| | `StaticTransformBroadcaster` | `TransformBroadcaster` |
|---|---|---|
| Topic | `/tf_static` | `/tf` |
| Publish frequency | Once at startup | Continuously at runtime |
| Latched | Yes | No |
| Use case | Fixed sensor mounts, URDF links | Robot pose, moving joints |

## Python Implementation

### Package Setup

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python learning_tf2_py \
    --dependencies rclpy geometry_msgs tf2_ros
```

### Node Code

Create `learning_tf2_py/static_turtle_tf2_broadcaster.py`:

```python
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf_transformations import quaternion_from_euler


class StaticFramePublisher(Node):

    def __init__(self, transformation):
        super().__init__('static_turtle_tf2_broadcaster')

        self._tf_publisher = StaticTransformBroadcaster(self)

        # Publish the static transform once at startup
        self.make_transforms(transformation)

    def make_transforms(self, transformation):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = transformation[1]    # parent frame
        t.child_frame_id  = transformation[2]    # child frame

        t.transform.translation.x = float(transformation[3])
        t.transform.translation.y = float(transformation[4])
        t.transform.translation.z = float(transformation[5])

        # Convert roll-pitch-yaw (radians) to quaternion
        quat = quaternion_from_euler(
            float(transformation[6]),   # roll
            float(transformation[7]),   # pitch
            float(transformation[8]))   # yaw

        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self._tf_publisher.sendTransform(t)


def main():
    rclpy.init()
    node = StaticFramePublisher(sys.argv)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
```

### `setup.py` entry point

```python
entry_points={
    'console_scripts': [
        'static_turtle_tf2_broadcaster = '
        'learning_tf2_py.static_turtle_tf2_broadcaster:main',
    ],
},
```

## C++ Implementation

### Node Code

Create `learning_tf2_cpp/src/static_turtle_tf2_broadcaster.cpp`:

```cpp
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

class StaticFramePublisher : public rclcpp::Node
{
public:
  explicit StaticFramePublisher(char * transformation[])
  : Node("static_turtle_tf2_broadcaster")
  {
    tf_static_broadcaster_ =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    make_transforms(transformation);
  }

private:
  void make_transforms(char * transformation[])
  {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = transformation[1];
    t.child_frame_id  = transformation[2];

    t.transform.translation.x = std::stod(transformation[3]);
    t.transform.translation.y = std::stod(transformation[4]);
    t.transform.translation.z = std::stod(transformation[5]);

    // Convert roll-pitch-yaw to quaternion using tf2
    tf2::Quaternion q;
    q.setRPY(
      std::stod(transformation[6]),   // roll
      std::stod(transformation[7]),   // pitch
      std::stod(transformation[8]));  // yaw

    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_static_broadcaster_->sendTransform(t);
  }

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StaticFramePublisher>(argv));
  rclcpp::shutdown();
  return 0;
}
```

## Building and Running

```bash
cd ~/ros2_ws
colcon build --packages-select learning_tf2_py
source install/setup.bash

# Start turtlesim
ros2 run turtlesim turtlesim_node &

# Broadcast a static transform: world → mystaticturtle
# Arguments: -- child_frame parent_frame x y z roll pitch yaw
ros2 run learning_tf2_py static_turtle_tf2_broadcaster \
  -- mystaticturtle world 0 1 0 0 0 0
```

## Verifying the Static Transform

```bash
# List all active frames (static frames appear here)
ros2 topic echo /tf_static

# Use tf2_echo to verify
ros2 run tf2_ros tf2_echo world mystaticturtle
```

Expected output:

```
At time 0.0
- Translation: [0.000, 1.000, 0.000]
- Rotation: in Quaternion [0.000, 0.000, 0.000, 1.000]
```

## The `robot_state_publisher` Alternative

For robot models described in URDF, you do **not** need to write static broadcasters manually. The `robot_state_publisher` node reads the URDF and automatically broadcasts all fixed joints as static transforms. Only write a `StaticTransformBroadcaster` when the transform cannot be expressed in a URDF.

## Next Steps

Proceed to [04_dynamic_broadcaster.md](04_dynamic_broadcaster.md) to learn how to broadcast time-varying transforms.
