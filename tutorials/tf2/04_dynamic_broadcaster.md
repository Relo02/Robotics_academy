# Tutorial 4 — Writing a Dynamic TF2 Broadcaster

> Reference: https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Py.html

## What is a Dynamic Broadcaster?

A **dynamic broadcaster** publishes transforms that change over time — for example, a robot's position in the world, a moving joint, or a tracked object. It publishes to the `/tf` topic at a regular rate (typically matching the sensor update rate or control loop).

Use a dynamic broadcaster when:
- The robot moves (odometry, SLAM pose)
- A joint rotates (arm, camera pan-tilt)
- An object is being tracked

## Python Implementation

### Node Code

Create `learning_tf2_py/turtle_tf2_broadcaster.py`:

```python
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from turtlesim.msg import Pose
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler


class FramePublisher(Node):

    def __init__(self):
        super().__init__('turtle_tf2_frame_publisher')

        # Declare which turtle to track as a ROS 2 parameter
        self.declare_parameter('turtlename', 'turtle')
        self.turtlename = self.get_parameter('turtlename').get_parameter_value().string_value

        # Create a TransformBroadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to the turtle's pose topic
        self.subscription = self.create_subscription(
            Pose,
            f'/{self.turtlename}/pose',
            self.handle_turtle_pose,
            1)

    def handle_turtle_pose(self, msg):
        t = TransformStamped()

        # Use the same timestamp as the incoming pose message
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = self.turtlename

        # The turtle lives in the XY plane — z is always 0
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0

        # Convert 2D heading (theta) to quaternion
        # In 2D: roll=0, pitch=0, yaw=theta
        q = quaternion_from_euler(0, 0, msg.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
```

## C++ Implementation

```cpp
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <turtlesim/msg/pose.hpp>

class FramePublisher : public rclcpp::Node
{
public:
  FramePublisher() : Node("turtle_tf2_frame_publisher")
  {
    this->declare_parameter<std::string>("turtlename", "turtle");
    turtlename_ = this->get_parameter("turtlename").as_string();

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    std::ostringstream stream;
    stream << "/" << turtlename_ << "/pose";
    subscription_ = this->create_subscription<turtlesim::msg::Pose>(
      stream.str(), 10,
      std::bind(&FramePublisher::handle_turtle_pose, this, std::placeholders::_1));
  }

private:
  void handle_turtle_pose(const std::shared_ptr<turtlesim::msg::Pose> msg)
  {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = turtlename_;

    t.transform.translation.x = msg->x;
    t.transform.translation.y = msg->y;
    t.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, msg->theta);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t);
  }

  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::string turtlename_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FramePublisher>());
  rclcpp::shutdown();
  return 0;
}
```

## Launch File

Create `learning_tf2_py/launch/turtle_tf2_demo.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='learning_tf2_py',
            executable='turtle_tf2_broadcaster',
            name='broadcaster1',
            parameters=[{'turtlename': 'turtle1'}]
        ),
    ])
```

## Building and Running

```bash
cd ~/ros2_ws
colcon build --packages-select learning_tf2_py
source install/setup.bash

# Launch turtlesim + broadcaster
ros2 launch learning_tf2_py turtle_tf2_demo.launch.py

# Drive turtle1 (in a new terminal)
ros2 run turtlesim turtle_teleop_key

# Watch the transform update in real time
ros2 run tf2_ros tf2_echo world turtle1
```

## The Euler → Quaternion Conversion

For a 2D rotation (yaw only), the quaternion is:

$$\mathbf{q} = \left(0,\ 0,\ \sin\frac{\theta}{2},\ \cos\frac{\theta}{2}\right)$$

More generally, for a rotation by angles $(\phi, \vartheta, \psi)$ in roll-pitch-yaw order:

$$q_x = \sin\frac{\phi}{2}\cos\frac{\vartheta}{2}\cos\frac{\psi}{2} - \cos\frac{\phi}{2}\sin\frac{\vartheta}{2}\sin\frac{\psi}{2}$$

$$q_y = \cos\frac{\phi}{2}\sin\frac{\vartheta}{2}\cos\frac{\psi}{2} + \sin\frac{\phi}{2}\cos\frac{\vartheta}{2}\sin\frac{\psi}{2}$$

$$q_z = \cos\frac{\phi}{2}\cos\frac{\vartheta}{2}\sin\frac{\psi}{2} - \sin\frac{\phi}{2}\sin\frac{\vartheta}{2}\cos\frac{\psi}{2}$$

$$q_w = \cos\frac{\phi}{2}\cos\frac{\vartheta}{2}\cos\frac{\psi}{2} + \sin\frac{\phi}{2}\sin\frac{\vartheta}{2}\sin\frac{\psi}{2}$$

The `quaternion_from_euler` function from `tf_transformations` (Python) and `tf2::Quaternion::setRPY` (C++) implement this automatically.

## Next Steps

Proceed to [05_listener.md](05_listener.md) to learn how to query and use transforms.
