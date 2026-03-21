# Tutorial 5 — Writing a TF2 Listener

> Reference: https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Py.html

## What is a TF2 Listener?

A **TF2 listener** subscribes to the `/tf` and `/tf_static` topics and stores all received transforms in a `Buffer`. You can then query the buffer for the transform between any two frames at any point in time within the buffer window (default: 10 seconds).

This is how `turtle2` knows where `turtle1` is relative to itself.

## Key API

| Class/Function | Description |
|----------------|-------------|
| `tf2_ros.Buffer` | Stores received transforms; thread-safe |
| `tf2_ros.TransformListener(buffer)` | Subscribes to `/tf` and fills the buffer |
| `buffer.lookup_transform(target, source, time)` | Returns the transform that maps points from `source` → `target` |
| `tf2_ros.LookupException` | Raised when the requested frames don't exist |
| `tf2_ros.ExtrapolationException` | Raised when the requested time is outside the buffer |

## Python Implementation

### Node Code

Create `learning_tf2_py/turtle_tf2_listener.py`:

```python
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from turtlesim.srv import Spawn


class FrameListener(Node):

    def __init__(self):
        super().__init__('turtle_tf2_frame_listener')

        # Create the TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Spawn turtle2
        self.spawner = self.create_client(Spawn, 'spawn')
        self.turtle_spawned = False

        # Publish velocity commands to turtle2
        self.publisher = self.create_publisher(Twist, 'turtle2/cmd_vel', 1)

        # Check the transform and send commands at 10 Hz
        self.timer = self.create_timer(0.1, self.on_timer)

    def on_timer(self):
        # Spawn turtle2 once on the first tick
        if not self.turtle_spawned:
            if self.spawner.service_is_ready():
                request = Spawn.Request()
                request.x = 4.0
                request.y = 2.0
                request.theta = 0.0
                request.name = 'turtle2'
                self.spawner.call_async(request)
                self.turtle_spawned = True
            return

        # Look up the transform: where is turtle1 relative to turtle2?
        try:
            t = self.tf_buffer.lookup_transform(
                'turtle2',          # target frame (we want everything in turtle2's frame)
                'turtle1',          # source frame
                rclpy.time.Time())  # latest available transform
        except TransformException as ex:
            self.get_logger().info(f'Could not transform turtle1 to turtle2: {ex}')
            return

        msg = Twist()

        # Scale factor for how aggressively turtle2 follows
        scale_rotation_rate = 1.0
        scale_forward_speed = 0.5

        # Angular velocity: steer toward turtle1
        msg.angular.z = scale_rotation_rate * math.atan2(
            t.transform.translation.y,
            t.transform.translation.x)

        # Linear velocity: proportional to distance
        msg.linear.x = scale_forward_speed * math.sqrt(
            t.transform.translation.x ** 2 +
            t.transform.translation.y ** 2)

        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
```

## C++ Implementation

```cpp
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "turtlesim/srv/spawn.hpp"

using namespace std::chrono_literals;

class FrameListener : public rclcpp::Node
{
public:
  FrameListener() : Node("turtle_tf2_frame_listener")
  {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    spawner_ = this->create_client<turtlesim::srv::Spawn>("spawn");
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle2/cmd_vel", 1);
    timer_ = this->create_wall_timer(100ms, std::bind(&FrameListener::on_timer, this));
  }

private:
  void on_timer()
  {
    if (!turtle_spawned_) {
      if (spawner_->service_is_ready()) {
        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->x = 4.0; request->y = 2.0; request->theta = 0.0;
        request->name = "turtle2";
        spawner_->async_send_request(request);
        turtle_spawned_ = true;
      }
      return;
    }

    geometry_msgs::msg::TransformStamped t;
    try {
      t = tf_buffer_->lookupTransform("turtle2", "turtle1", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform: %s", ex.what());
      return;
    }

    geometry_msgs::msg::Twist msg;
    msg.angular.z = 1.0 * atan2(t.transform.translation.y, t.transform.translation.x);
    msg.linear.x  = 0.5 * sqrt(pow(t.transform.translation.x, 2) +
                                pow(t.transform.translation.y, 2));
    publisher_->publish(msg);
  }

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawner_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool turtle_spawned_ = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrameListener>());
  rclcpp::shutdown();
  return 0;
}
```

## Understanding `lookup_transform`

```python
t = self.tf_buffer.lookup_transform(
    target_frame,   # express the result IN this frame
    source_frame,   # transform FROM this frame
    time)           # at this time (rclpy.time.Time() = latest)
```

The returned `TransformStamped` tells you: **"where is the origin of `source_frame` expressed in `target_frame`?"**

In the turtle example, `lookup_transform('turtle2', 'turtle1', ...)` returns the position of `turtle1`'s origin as seen from `turtle2`'s coordinate frame. This is exactly what the follower needs to compute a steering command.

## Time Parameter Options

| Value | Meaning |
|-------|---------|
| `rclpy.time.Time()` | Latest available transform (most common) |
| `self.get_clock().now()` | Current time (may raise `ExtrapolationException` if data is slightly stale) |
| `rclpy.time.Time(seconds=t)` | Specific past timestamp |

## Error Handling

Always wrap `lookup_transform` in a try/except:

```python
try:
    t = self.tf_buffer.lookup_transform('turtle2', 'turtle1', rclpy.time.Time())
except tf2_ros.LookupException as e:
    # One of the frames doesn't exist yet
    self.get_logger().warn(str(e))
except tf2_ros.ExtrapolationException as e:
    # Requested time is outside the buffer window
    self.get_logger().warn(str(e))
```

## Building and Running the Complete Demo

```bash
cd ~/ros2_ws && colcon build --packages-select learning_tf2_py
source install/setup.bash

# Launch broadcaster for turtle1 + turtlesim
ros2 launch learning_tf2_py turtle_tf2_demo.launch.py

# Run the listener (spawns turtle2 and starts following)
ros2 run learning_tf2_py turtle_tf2_listener

# Drive turtle1
ros2 run turtlesim turtle_teleop_key
```

## Next Steps

Proceed to [06_tools_and_debugging.md](06_tools_and_debugging.md) to learn the CLI tools for inspecting and debugging TF2.
