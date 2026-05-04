#!/usr/bin/env python3
"""Relay /cmd_vel (Nav2 output) → /diff_drive_controller/cmd_vel_unstamped."""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__("cmd_vel_relay")
        self.pub = self.create_publisher(
            Twist, "/diff_drive_controller/cmd_vel_unstamped", 10
        )
        self.create_subscription(Twist, "/cmd_vel", self.pub.publish, 10)


def main():
    rclpy.init()
    rclpy.spin(CmdVelRelay())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
