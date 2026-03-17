#!/usr/bin/env python3
import math
import random
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from mower_msgs.msg import UltrasonicArray
from visualization_msgs.msg import Marker, MarkerArray


_SENSOR_POSES = [
    (0.35, 0.15, 0.785),   # 0 front_left  (45° forward-left)
    (0.40, 0.00, 0.000),   # 1 front_center (straight ahead)
    (0.35, -0.15, -0.785), # 2 front_right (45° forward-right)
    (0.00, 0.25, 1.5708),  # 3 left  (90° left)
    (0.00, -0.25, -1.5708),# 4 right (90° right)
    (-0.35, 0.00, 3.14159) # 5 rear  (180°)
]


def _ray_circle_distance(
    sx: float,
    sy: float,
    dx: float,
    dy: float,
    cx: float,
    cy: float,
    radius: float,
    min_range: float,
    max_range: float,
) -> Optional[float]:
    fx = sx - cx
    fy = sy - cy
    b = 2.0 * (fx * dx + fy * dy)
    c = fx * fx + fy * fy - radius * radius
    disc = b * b - 4.0 * c
    if disc < 0.0:
        return None
    sqrt_disc = math.sqrt(disc)
    t1 = (-b - sqrt_disc) * 0.5
    t2 = (-b + sqrt_disc) * 0.5
    t = t1 if t1 >= min_range else t2
    if t < min_range or t > max_range:
        return None
    return t


class MockBaseInterfaceNode(Node):
    def __init__(self):
        super().__init__('mock_base_interface_node')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('ultrasonic_clear_range_m', 2.0)
        self.declare_parameter('ultrasonic_min_range_m', 0.02)
        self.declare_parameter('ultrasonic_max_range_m', 4.0)
        self.declare_parameter('simulate_obstacle_range_m', -1.0)
        self.declare_parameter('virtual_obstacle_center_x', 0.0)
        self.declare_parameter('virtual_obstacle_center_y', 0.0)
        self.declare_parameter('virtual_obstacle_radius_m', -1.0)
        # Optional second virtual obstacle circle (used for RViz mission testing).
        self.declare_parameter('virtual_obstacle2_center_x', 0.0)
        self.declare_parameter('virtual_obstacle2_center_y', 0.0)
        self.declare_parameter('virtual_obstacle2_radius_m', -1.0)
        self.declare_parameter('virtual_obstacle3_center_x', 0.0)
        self.declare_parameter('virtual_obstacle3_center_y', 0.0)
        self.declare_parameter('virtual_obstacle3_radius_m', 0.0)
        # Optional initial pose override (useful for RViz mission testing).
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw_rad', 0.0)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('publish_ultrasonic', True)
        self.declare_parameter('publish_odom', True)
        # Sensor noise test (simulation layer only): realistic noise, false obstacles, dropouts.
        self.declare_parameter('sensor_noise_test', False)
        self.declare_parameter('ultrasonic_noise_std_m', 0.05)
        self.declare_parameter('ultrasonic_false_obstacle_prob', 0.02)
        self.declare_parameter('ultrasonic_false_obstacle_range_m', 0.4)
        self.declare_parameter('ultrasonic_dropout_prob', 0.01)
        self.declare_parameter('ultrasonic_dropout_max_ticks', 2)

        self.publish_rate_hz = self.get_parameter('publish_rate_hz').value
        self.ultrasonic_clear = self.get_parameter('ultrasonic_clear_range_m').value
        v = self.get_parameter('simulate_obstacle_range_m').value
        self.simulate_obstacle_range_m = float(v) if isinstance(v, str) else v
        self.ultrasonic_min = self.get_parameter('ultrasonic_min_range_m').value
        self.ultrasonic_max = self.get_parameter('ultrasonic_max_range_m').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        p = self.get_parameter('publish_ultrasonic').value
        self.publish_ultrasonic = p if isinstance(p, bool) else (str(p).lower() == 'true')
        po = self.get_parameter('publish_odom').value
        self.publish_odom = po if isinstance(po, bool) else (str(po).lower() == 'true')

        # Base virtual obstacle parameters (used as a starting point, then repositioned
        # for the straight-line multi-obstacle test).
        self.virtual_obstacle_cx = float(self.get_parameter('virtual_obstacle_center_x').value)
        self.virtual_obstacle_cy = float(self.get_parameter('virtual_obstacle_center_y').value)
        self.virtual_obstacle_radius = float(self.get_parameter('virtual_obstacle_radius_m').value)
        self.virtual_obstacle2_cx = float(self.get_parameter('virtual_obstacle2_center_x').value)
        self.virtual_obstacle2_cy = float(self.get_parameter('virtual_obstacle2_center_y').value)
        self.virtual_obstacle2_radius = float(self.get_parameter('virtual_obstacle2_radius_m').value)
        self.sequence_obstacle3_cx = float(self.get_parameter('virtual_obstacle3_center_x').value)
        self.sequence_obstacle3_cy = float(self.get_parameter('virtual_obstacle3_center_y').value)
        self.sequence_obstacle3_radius = float(self.get_parameter('virtual_obstacle3_radius_m').value)

        # Initial pose (can be overridden via parameters; defaults to origin).
        self.x = float(self.get_parameter('initial_x').value)
        self.y = float(self.get_parameter('initial_y').value)
        self.yaw = float(self.get_parameter('initial_yaw_rad').value)

        self.stop_asserted = False
        self.last_cmd_vel = Twist()
        # Sensor noise test state
        sn = self.get_parameter('sensor_noise_test').value
        self.sensor_noise_test = sn if isinstance(sn, bool) else (str(sn).lower() == 'true')
        if self.sensor_noise_test:
            self.ultrasonic_noise_std = float(self.get_parameter('ultrasonic_noise_std_m').value)
            self.ultrasonic_false_obstacle_prob = float(
                self.get_parameter('ultrasonic_false_obstacle_prob').value
            )
            self.ultrasonic_false_obstacle_range = float(
                self.get_parameter('ultrasonic_false_obstacle_range_m').value
            )
            self.ultrasonic_dropout_prob = float(
                self.get_parameter('ultrasonic_dropout_prob').value
            )
            self.ultrasonic_dropout_max_ticks = int(
                self.get_parameter('ultrasonic_dropout_max_ticks').value
            )
            self.dropout_ticks_remaining = 0
            self.get_logger().info(
                'Sensor noise test ON: noise_std=%.3f false_obstacle_prob=%.3f dropout_prob=%.3f' % (
                    self.ultrasonic_noise_std,
                    self.ultrasonic_false_obstacle_prob,
                    self.ultrasonic_dropout_prob,
                )
            )

        self.heartbeat_pub = self.create_publisher(Empty, '/base/heartbeat', 10)
        if self.publish_odom:
            self.odom_pub = self.create_publisher(Odometry, '/odom/raw', 10)
        else:
            self.odom_pub = None
        if self.publish_ultrasonic:
            self.ultrasonic_pub = self.create_publisher(UltrasonicArray, '/ultrasonic/ranges', 10)
        else:
            self.ultrasonic_pub = None
        # RViz obstacle debug markers: publish cubes matching virtual obstacles
        # to the same topic used by the waypoint follower for waypoints.
        self.obstacle_markers_pub = self.create_publisher(
            MarkerArray, '/mission/waypoints_markers', 1
        )
        self.create_subscription(Bool, '/safety/stop', self._cb_safety_stop, 10)
        self.create_subscription(Twist, '/cmd_vel', self._cb_cmd_vel, 10)

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._timer_cb)
        self.markers_timer = self.create_timer(1.0, self._publish_obstacle_markers)
        self.get_logger().info('Mock base interface: publish_ultrasonic=%s publish_odom=%s' % (
            'true' if self.publish_ultrasonic else 'false',
            'true' if self.publish_odom else 'false'))

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
        if self.odom_pub is not None:
            self.odom_pub.publish(odom)

        # Ultrasonic: clear distances, simple front obstacle, or virtual circle obstacle
        if self.publish_ultrasonic and self.ultrasonic_pub is not None:
            ranges = [float(self.ultrasonic_clear)] * 6
            if self.simulate_obstacle_range_m > 0.0:
                ranges[0] = ranges[1] = ranges[2] = float(self.simulate_obstacle_range_m)
            elif self.virtual_obstacle_radius > 0.0 or self.virtual_obstacle2_radius > 0.0:
                ranges = self._virtual_obstacle_ranges()
            # Sensor noise test: inject noise, occasional false obstacles, and short dropouts
            if getattr(self, 'sensor_noise_test', False):
                ranges = self._apply_sensor_noise(ranges)
                if getattr(self, 'dropout_ticks_remaining', 0) > 0:
                    self.dropout_ticks_remaining -= 1
                    # Skip publish this tick (brief dropout; guard will see timeout and allow motion)
                    return
                if random.random() < self.ultrasonic_dropout_prob:
                    self.dropout_ticks_remaining = random.randint(
                        1, max(1, self.ultrasonic_dropout_max_ticks)
                    )
            ua = UltrasonicArray()
            ua.header.stamp = now
            ua.header.frame_id = self.base_frame_id
            ua.range_m = ranges
            ua.min_range_m = float(self.ultrasonic_min)
            ua.max_range_m = float(self.ultrasonic_max)
            self.ultrasonic_pub.publish(ua)

    def _apply_sensor_noise(self, ranges: list) -> list:
        """Apply Gaussian noise and occasional false obstacle (simulation layer only)."""
        out = []
        for i, r in enumerate(ranges):
            noise = random.gauss(0.0, self.ultrasonic_noise_std)
            v = float(r) + noise
            v = max(self.ultrasonic_min, min(self.ultrasonic_max, v))
            out.append(v)
        # Occasional false obstacle: one front sensor reports a short range
        if random.random() < self.ultrasonic_false_obstacle_prob:
            idx = random.randint(0, 2)  # 0=front_left, 1=front_center, 2=front_right
            out[idx] = max(
                self.ultrasonic_min,
                min(self.ultrasonic_max, self.ultrasonic_false_obstacle_range),
            )
        return out

    def _publish_obstacle_markers(self):
        if self.obstacle_markers_pub is None:
            return
        array = MarkerArray()
        now = self.get_clock().now().to_msg()

        # Obstacles are defined in the odom frame for the straight-line sim test.
        # Use distinct colors for each obstacle to aid debugging.
        def make_cube(idx, cx, cy, radius, color):
            m = Marker()
            m.header.stamp = now
            m.header.frame_id = self.odom_frame_id
            m.ns = 'debug_obstacle'
            m.id = idx
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = float(cx)
            m.pose.position.y = float(cy)
            m.pose.position.z = 0.35
            m.scale.x = float(radius) * 2.0
            m.scale.y = float(radius) * 2.0
            m.scale.z = 0.7
            m.color.r, m.color.g, m.color.b, m.color.a = color
            return m

        # Clear only our own obstacle namespace; waypoint markers use a different namespace.
        clear = Marker()
        clear.header.stamp = now
        clear.header.frame_id = self.odom_frame_id
        clear.ns = 'debug_obstacle'
        clear.id = 0
        clear.action = Marker.DELETEALL
        array.markers.append(clear)

        markers = []
        if self.virtual_obstacle_radius > 0.0:
            markers.append(
                make_cube(0, self.virtual_obstacle_cx, self.virtual_obstacle_cy,
                          self.virtual_obstacle_radius, (1.0, 0.15, 0.15, 1.0))
            )
        if self.virtual_obstacle2_radius > 0.0:
            markers.append(
                make_cube(1, self.virtual_obstacle2_cx, self.virtual_obstacle2_cy,
                          self.virtual_obstacle2_radius, (1.0, 0.45, 0.0, 1.0))
            )
        if getattr(self, 'sequence_obstacle3_radius', 0.0) > 0.0:
            markers.append(
                make_cube(2, self.sequence_obstacle3_cx, self.sequence_obstacle3_cy,
                          self.sequence_obstacle3_radius, (0.2, 0.8, 0.2, 1.0))
            )

        array.markers.extend(markers)
        self.obstacle_markers_pub.publish(array)

    def _virtual_obstacle_ranges(self):
        ranges = [float(self.ultrasonic_clear)] * 6
        rx = self.x
        ry = self.y
        ryaw = self.yaw
        cos_r = math.cos(ryaw)
        sin_r = math.sin(ryaw)
        for i, (lx, ly, lyaw) in enumerate(_SENSOR_POSES):
            wx = rx + lx * cos_r - ly * sin_r
            wy = ry + lx * sin_r + ly * cos_r
            wyaw = ryaw + lyaw
            dx = math.cos(wyaw)
            dy = math.sin(wyaw)
            best = None
            if self.virtual_obstacle_radius > 0.0:
                dist1 = _ray_circle_distance(
                    wx,
                    wy,
                    dx,
                    dy,
                    self.virtual_obstacle_cx,
                    self.virtual_obstacle_cy,
                    self.virtual_obstacle_radius,
                    self.ultrasonic_min,
                    self.ultrasonic_max,
                )
                if dist1 is not None:
                    best = dist1
            if self.virtual_obstacle2_radius > 0.0:
                dist2 = _ray_circle_distance(
                    wx,
                    wy,
                    dx,
                    dy,
                    self.virtual_obstacle2_cx,
                    self.virtual_obstacle2_cy,
                    self.virtual_obstacle2_radius,
                    self.ultrasonic_min,
                    self.ultrasonic_max,
                )
                if dist2 is not None:
                    best = dist2 if best is None else min(best, dist2)
            if self.sequence_obstacle3_radius > 0.0:
                dist3 = _ray_circle_distance(
                    wx,
                    wy,
                    dx,
                    dy,
                    self.sequence_obstacle3_cx,
                    self.sequence_obstacle3_cy,
                    self.sequence_obstacle3_radius,
                    self.ultrasonic_min,
                    self.ultrasonic_max,
                )
                if dist3 is not None:
                    best = dist3 if best is None else min(best, dist3)
            if best is not None:
                ranges[i] = best
        return ranges

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
