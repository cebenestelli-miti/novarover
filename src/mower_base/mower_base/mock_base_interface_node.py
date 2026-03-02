#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from mower_msgs.msg import UltrasonicArray


class MockBaseInterfaceNode(Node):
    def __init__(self):
        super().__init__('mock_base_interface_node')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('ultrasonic_clear_range_m', 2.0)
        self.declare_parameter('ultrasonic_min_range_m', 0.02)
        self.declare_parameter('ultrasonic_max_range_m', 4.0)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('publish_ultrasonic', True)

        self.publish_rate_hz = self.get_parameter('publish_rate_hz').value
        self.ultrasonic_clear = self.get_parameter('ultrasonic_clear_range_m').value
        self.ultrasonic_min = self.get_parameter('ultrasonic_min_range_m').value
        self.ultrasonic_max = self.get_parameter('ultrasonic_max_range_m').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        p = self.get_parameter('publish_ultrasonic').value
        self.publish_ultrasonic = p if isinstance(p, bool) else (str(p).lower() == 'true')

        self.stop_asserted = False
        self.last_cmd_vel = Twist()
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.heartbeat_pub = self.create_publisher(Empty, '/base/heartbeat', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom/raw', 10)
        if self.publish_ultrasonic:
            self.ultrasonic_pub = self.create_publisher(UltrasonicArray, '/ultrasonic/ranges', 10)
        else:
            self.ultrasonic_pub = None
        self.create_subscription(Bool, '/safety/stop', self._cb_safety_stop, 10)
        self.create_subscription(Twist, '/cmd_vel', self._cb_cmd_vel, 10)

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._timer_cb)
        self.get_logger().info('Mock base interface: publish_ultrasonic=%s' % ('true' if self.publish_ultrasonic else 'false'))

    def _cb_safety_stop(self, msg):
        self.stop_asserted = msg.data

    def _cb_cmd_vel(self, msg):
        self.last_cmd_vel = msg

    def _timer_cb(self):
        now = self.get_clock().now().to_msg()
        # Heartbeat
        self.heartbeat_pub.publish(Empty())

        # Odometry: integrate cmd_vel when NOT stopped; when stopped treat as zero
        dt = 1.0 / self.publish_rate_hz
        if self.stop_asserted:
            vx = 0.0
            vz = 0.0
        else:
            vx = self.last_cmd_vel.linear.x
            vz = self.last_cmd_vel.angular.z
        self.x += vx * dt * math.cos(self.yaw)
        self.y += vx * dt * math.sin(self.yaw)
        self.yaw += vz * dt

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        q = self._yaw_to_quaternion(self.yaw)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = vz
        self.odom_pub.publish(odom)

        # Ultrasonic: clear distances (only if publish_ultrasonic)
        if self.publish_ultrasonic and self.ultrasonic_pub is not None:
            ranges = [float(self.ultrasonic_clear)] * 6
            ua = UltrasonicArray()
            ua.header.stamp = now
            ua.header.frame_id = self.base_frame_id
            ua.range_m = ranges
            ua.min_range_m = float(self.ultrasonic_min)
            ua.max_range_m = float(self.ultrasonic_max)
            self.ultrasonic_pub.publish(ua)

    @staticmethod
    def _yaw_to_quaternion(yaw):
        return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def main(args=None):
    rclpy.init(args=args)
    node = MockBaseInterfaceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
