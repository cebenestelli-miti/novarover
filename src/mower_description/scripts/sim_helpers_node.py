#!/usr/bin/env python3
"""
Sim helper node for Gazebo:

- Publishes /base/heartbeat (10 Hz)
- Publishes /mower/stall (1 Hz, false)
- Aggregates 6 ultrasonic scans into /ultrasonic/ranges (UltrasonicArray)
- Gates /cmd_vel using /safety/stop and publishes /cmd_vel_gz for Gazebo
"""

import math
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from std_msgs.msg import Empty, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from mower_msgs.msg import UltrasonicArray


class SimHelpersNode(Node):
    def __init__(self) -> None:
        super().__init__("sim_helpers_node")

        # Parameters for ultrasonic aggregation
        self.declare_parameter("ultrasonic_min_range_m", 0.02)
        self.declare_parameter("ultrasonic_max_range_m", 4.0)
        self.declare_parameter("ultrasonic_timeout_sec", 0.5)

        self.ultrasonic_min_range = float(
            self.get_parameter("ultrasonic_min_range_m").value
        )
        self.ultrasonic_max_range = float(
            self.get_parameter("ultrasonic_max_range_m").value
        )
        self.ultrasonic_timeout_sec = float(
            self.get_parameter("ultrasonic_timeout_sec").value
        )

        # Heartbeat / stall
        self.heartbeat_pub = self.create_publisher(Empty, "/base/heartbeat", 10)
        self.stall_pub = self.create_publisher(Bool, "/mower/stall", 10)

        # Ultrasonic aggregation
        self.ultrasonic_pub = self.create_publisher(
            UltrasonicArray, "/ultrasonic/ranges", 10
        )
        self._last_scans: List[Optional[LaserScan]] = [None] * 6
        self._last_scan_times: List[Optional[Time]] = [None] * 6

        # Topic names must not end with a numeric token alone (ROS 2 naming rule),
        # so we use scan0..scan5 instead of scan/0..scan/5.
        scan_topics = [f"/ultrasonic/scan{i}" for i in range(6)]
        for idx, topic in enumerate(scan_topics):
            self.create_subscription(
                LaserScan,
                topic,
                lambda msg, i=idx: self._on_scan(i, msg),
                10,
            )

        # Cmd_vel gating
        self.cmd_vel_safe_pub = self.create_publisher(Twist, "/cmd_vel_gz", 10)
        self._last_cmd_vel: Optional[Twist] = None
        self._safety_stop: bool = False

        self.create_subscription(
            Twist, "/cmd_vel", self._on_cmd_vel, 10
        )
        self.create_subscription(
            Bool, "/safety/stop", self._on_safety_stop, 10
        )

        # Timers
        self.create_timer(0.1, self._publish_heartbeat)  # 10 Hz
        self.create_timer(1.0, self._publish_stall)      # 1 Hz
        self.create_timer(0.1, self._publish_ultrasonic) # 10 Hz
        self.create_timer(0.05, self._publish_cmd_vel)   # 20 Hz

        self.get_logger().info(
            "Sim helpers running: heartbeat, stall, ultrasonic aggregation, cmd_vel gating"
        )

    # --- Heartbeat / stall ---
    def _publish_heartbeat(self) -> None:
        self.heartbeat_pub.publish(Empty())

    def _publish_stall(self) -> None:
        msg = Bool()
        msg.data = False
        self.stall_pub.publish(msg)

    # --- Ultrasonic aggregation ---
    def _on_scan(self, index: int, msg: LaserScan) -> None:
        if 0 <= index < 6:
            self._last_scans[index] = msg
            self._last_scan_times[index] = self.get_clock().now()

    def _publish_ultrasonic(self) -> None:
        now = self.get_clock().now()
        ua = UltrasonicArray()
        ua.header.stamp = now.to_msg()
        ua.range_m = [math.nan] * 6
        ua.min_range_m = float(self.ultrasonic_min_range)
        ua.max_range_m = float(self.ultrasonic_max_range)

        for i in range(6):
            scan = self._last_scans[i]
            t = self._last_scan_times[i]
            if scan is None or t is None:
                continue

            dt = (now - t).nanoseconds / 1e9
            if dt > self.ultrasonic_timeout_sec:
                continue

            min_valid = None
            for r in scan.ranges:
                if not math.isfinite(r):
                    continue
                if r < self.ultrasonic_min_range or r > self.ultrasonic_max_range:
                    continue
                if min_valid is None or r < min_valid:
                    min_valid = r

            if min_valid is not None:
                ua.range_m[i] = float(min_valid)

        self.ultrasonic_pub.publish(ua)

    # --- Cmd_vel gating ---
    def _on_cmd_vel(self, msg: Twist) -> None:
        self._last_cmd_vel = msg

    def _on_safety_stop(self, msg: Bool) -> None:
        self._safety_stop = bool(msg.data)

    def _publish_cmd_vel(self) -> None:
        out = Twist()
        if self._last_cmd_vel is not None and not self._safety_stop:
            out = self._last_cmd_vel
        # else: remain zero
        self.cmd_vel_safe_pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimHelpersNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
