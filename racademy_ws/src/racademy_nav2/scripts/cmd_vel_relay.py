#!/usr/bin/env python3
"""Relay Nav2 Twist commands to the diff-drive controller command topic."""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelRelay(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_relay")

        self.declare_parameter("input_topic", "/cmd_vel")
        self.declare_parameter("output_topic", "/diff_drive_controller/cmd_vel_unstamped")

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value

        self.publisher = self.create_publisher(Twist, output_topic, 10)
        self.subscription = self.create_subscription(Twist, input_topic, self._on_cmd_vel, 10)

        self.get_logger().info(f"Relaying Twist from {input_topic} -> {output_topic}")

    def _on_cmd_vel(self, msg: Twist) -> None:
        self.publisher.publish(msg)


def main() -> None:
    rclpy.init()
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
