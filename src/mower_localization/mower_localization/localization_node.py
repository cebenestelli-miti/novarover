#!/usr/bin/env python3
"""
Minimal localization node per architecture §3.

Subscribes to /odom/raw (and optionally /gps/fix, /imu/data for future fusion).
Publishes filtered odometry on /odom and TF odom → base_link.

Placeholder: forwards /odom/raw to /odom and broadcasts the same pose as TF.
No new topics renamed; existing topic contracts unchanged.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class LocalizationNode(Node):
    def __init__(self):
        super().__init__('localization_node')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')

        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value

        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.create_subscription(Odometry, '/odom/raw', self._cb_odom_raw, 10)

        # Optional inputs for future fusion (no-op for now)
        # self.create_subscription(NavSatFix, '/gps/fix', self._cb_gps, 10)
        # self.create_subscription(Imu, '/imu/data', self._cb_imu, 10)

        self.get_logger().info(
            'Localization node: /odom/raw -> /odom + TF %s -> %s' % (self.odom_frame_id, self.base_frame_id)
        )

    def _cb_odom_raw(self, msg):
        # Publish filtered odom (placeholder: same as raw)
        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.pose = msg.pose
        out.twist = msg.twist
        # Normalize frame_ids to our params for consistency
        out.header.frame_id = self.odom_frame_id
        out.child_frame_id = self.base_frame_id
        self.odom_pub.publish(out)

        # Broadcast TF odom -> base_link
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.odom_frame_id
        t.child_frame_id = self.base_frame_id
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
