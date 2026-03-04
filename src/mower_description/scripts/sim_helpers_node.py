#!/usr/bin/env python3
"""Publishes /base/heartbeat and /mower/stall for Gazebo sim (topic contract)."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Bool


def main(args=None):
    rclpy.init(args=args)
    node = Node("sim_helpers_node")
    heartbeat_pub = node.create_publisher(Empty, "/base/heartbeat", 10)
    stall_pub = node.create_publisher(Bool, "/mower/stall", 10)
    node.get_logger().info("Sim helpers: publishing /base/heartbeat (10 Hz), /mower/stall (1 Hz, false)")

    timer_10hz = node.create_timer(0.1, lambda: heartbeat_pub.publish(Empty()))
    stall_msg = Bool(data=False)
    timer_stall = node.create_timer(1.0, lambda: stall_pub.publish(stall_msg))

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
