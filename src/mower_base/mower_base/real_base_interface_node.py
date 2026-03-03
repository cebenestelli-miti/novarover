#!/usr/bin/env python3
"""
Real base interface (Arduino bridge) per architecture §3.

Topic contract:
  Subscribes: /cmd_vel, /safety/stop, /engine/cmd
  Publishes:  /base/heartbeat, /odom/raw, /ultrasonic/ranges, /mower/stall

This node is a STUB until the Arduino is connected. It publishes heartbeat and
zero odom so the rest of the stack (safety_manager, localization) does not
time out. Replace the _read_arduino() / _write_arduino() logic with your
serial protocol when hardware is ready.
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from mower_msgs.msg import UltrasonicArray, EngineCmd


class RealBaseInterfaceNode(Node):
    def __init__(self):
        super().__init__('real_base_interface_node')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('publish_ultrasonic', True)  # publish /ultrasonic/ranges when Arduino provides them
        self.declare_parameter('serial_port', '')  # e.g. /dev/ttyUSB0; empty = stub mode (no Arduino)
        self.declare_parameter('publish_odom', True)  # set false when Gazebo/bridge provides /odom/raw

        self.rate_hz = self.get_parameter('publish_rate_hz').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        p = self.get_parameter('publish_ultrasonic').value
        self.publish_ultrasonic = p if isinstance(p, bool) else (str(p).lower() == 'true')
        self.serial_port = self.get_parameter('serial_port').value
        p_odom = self.get_parameter('publish_odom').value
        self.publish_odom = p_odom if isinstance(p_odom, bool) else (str(p_odom).lower() == 'true')

        self.stop_asserted = False
        self.last_cmd_vel = Twist()
        self.last_engine_cmd = None

        # State from Arduino (stub: zeros / no stall)
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.odom_vx = 0.0
        self.odom_vz = 0.0
        self.ultrasonic_ranges = [2.0] * 6
        self.stall = False

        # Publishers
        self.heartbeat_pub = self.create_publisher(Empty, '/base/heartbeat', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom/raw', 10) if self.publish_odom else None
        if self.publish_ultrasonic:
            self.ultrasonic_pub = self.create_publisher(UltrasonicArray, '/ultrasonic/ranges', 10)
        else:
            self.ultrasonic_pub = None
        self.stall_pub = self.create_publisher(Bool, '/mower/stall', 10)

        # Subscribers
        self.create_subscription(Bool, '/safety/stop', self._cb_safety_stop, 10)
        self.create_subscription(Twist, '/cmd_vel', self._cb_cmd_vel, 10)
        self.create_subscription(EngineCmd, '/engine/cmd', self._cb_engine_cmd, 10)

        self.timer = self.create_timer(1.0 / self.rate_hz, self._timer_cb)

        if not self.serial_port:
            self.get_logger().warn(
                'Real base interface: serial_port empty — STUB MODE (heartbeat + zero odom). '
                'Set serial_port (e.g. /dev/ttyUSB0) and implement Arduino protocol for hardware.'
            )
        else:
            self.get_logger().info('Real base interface: serial_port=%s' % self.serial_port)
            # TODO: open serial connection to Arduino

    def _cb_safety_stop(self, msg):
        self.stop_asserted = msg.data
        # TODO: send to Arduino so it can enforce stop locally

    def _cb_cmd_vel(self, msg):
        self.last_cmd_vel = msg
        # TODO: when Arduino connected, send cmd_vel (gated by safety/stop) to Arduino

    def _cb_engine_cmd(self, msg):
        self.last_engine_cmd = msg
        # TODO: when Arduino connected, send engine/blade commands to Arduino

    def _read_arduino(self):
        """Read one cycle of data from Arduino (odom, ultrasonics, stall). Stub: no-op."""
        # TODO: parse serial message from Arduino; update self.odom_*, self.ultrasonic_ranges, self.stall
        pass

    def _write_arduino(self):
        """Send cmd_vel (if not stopped) and engine_cmd to Arduino. Stub: no-op."""
        # TODO: send last_cmd_vel (when not stop_asserted) and last_engine_cmd over serial
        pass

    def _timer_cb(self):
        now = self.get_clock().now().to_msg()

        # Exchange with Arduino when serial is configured
        if self.serial_port:
            self._read_arduino()
            self._write_arduino()
        # else: keep stub state (zeros)

        # Heartbeat (so safety_manager does not time out)
        self.heartbeat_pub.publish(Empty())

        # Odometry (skip when Gazebo/bridge provides /odom/raw)
        if self.odom_pub is not None:
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = self.odom_frame_id
            odom.child_frame_id = self.base_frame_id
            odom.pose.pose.position.x = self.odom_x
            odom.pose.pose.position.y = self.odom_y
            odom.pose.pose.position.z = 0.0
            q = self._yaw_to_quaternion(self.odom_yaw)
            odom.pose.pose.orientation.x = q[0]
            odom.pose.pose.orientation.y = q[1]
            odom.pose.pose.orientation.z = q[2]
            odom.pose.pose.orientation.w = q[3]
            odom.twist.twist.linear.x = self.odom_vx
            odom.twist.twist.linear.y = 0.0
            odom.twist.twist.linear.z = 0.0
            odom.twist.twist.angular.x = 0.0
            odom.twist.twist.angular.y = 0.0
            odom.twist.twist.angular.z = self.odom_vz
            self.odom_pub.publish(odom)

        # Ultrasonics (if enabled)
        if self.publish_ultrasonic and self.ultrasonic_pub is not None:
            ua = UltrasonicArray()
            ua.header.stamp = now
            ua.header.frame_id = self.base_frame_id
            ua.range_m = list(self.ultrasonic_ranges)
            ua.min_range_m = 0.02
            ua.max_range_m = 4.0
            self.ultrasonic_pub.publish(ua)

        # Stall
        stall_msg = Bool()
        stall_msg.data = self.stall
        self.stall_pub.publish(stall_msg)

    @staticmethod
    def _yaw_to_quaternion(yaw):
        return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def main(args=None):
    rclpy.init(args=args)
    node = RealBaseInterfaceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
