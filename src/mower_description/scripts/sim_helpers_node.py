#!/usr/bin/env python3
"""
Sim helper node for Gazebo:

- Publishes /base/heartbeat (10 Hz)
- Publishes /mower/stall (1 Hz, false)
- Computes /ultrasonic/ranges from robot pose + known obstacle positions (10 Hz)
  Uses ray-circle intersection so it works in WSL2 without GPU lidar rendering.
  Falls back to Gazebo gpu_lidar scan aggregation if pose is unavailable.
- Gates /cmd_vel using /safety/stop and publishes /cmd_vel_gz for Gazebo
"""

import math
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from std_msgs.msg import Empty, Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from mower_msgs.msg import UltrasonicArray

# Sensor relative poses in robot/base_link frame: (x_m, y_m, yaw_rad)
# Indices: 0=front_left, 1=front_center, 2=front_right, 3=left, 4=right, 5=rear
_SENSOR_POSES: List[Tuple[float, float, float]] = [
    (0.35,  0.15,  0.785),   # 0 front_left  (45° forward-left)
    (0.40,  0.00,  0.000),   # 1 front_center (straight ahead)
    (0.35, -0.15, -0.785),   # 2 front_right (45° forward-right)
    (0.00,  0.25,  1.5708),  # 3 left  (90° left)
    (0.00, -0.25, -1.5708),  # 4 right (90° right)
    (-0.35, 0.00,  3.14159), # 5 rear  (180°)
]


def _ray_circle_distance(
    sx: float, sy: float, dx: float, dy: float,
    cx: float, cy: float, radius: float,
    min_range: float, max_range: float,
) -> Optional[float]:
    """
    Ray-circle intersection. Returns the distance from the ray origin to the
    nearest hit on the circle surface, or None if no hit within [min_range, max_range].

    Ray: origin (sx, sy), unit direction (dx, dy).
    Circle: centre (cx, cy), radius.
    """
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


class SimHelpersNode(Node):
    def __init__(self) -> None:
        super().__init__("sim_helpers_node")

        # --- Parameters ---
        self.declare_parameter("cmd_vel_in_topic", "/cmd_vel")
        self.declare_parameter("ultrasonic_min_range_m", 0.25)
        self.declare_parameter("ultrasonic_max_range_m", 4.0)
        self.declare_parameter("ultrasonic_timeout_sec", 0.5)
        # Obstacle list: flat [x0, y0, r0, x1, y1, r1, ...] in odom/world frame.
        # Default matches the obstacle box in mower_empty.sdf (0.5x0.5 box → radius ~0.36m,
        # placed at x=5 so the robot has full detection range for WP0→WP1).
        # radius=0.25m matches the box half-side (0.5m box → 0.25m half-side).
        # 0.36m (half-diagonal) over-predicted the north/south faces, causing
        # spurious stop_request when passing the obstacle laterally.
        self.declare_parameter("obstacle_positions", [5.0, 0.0, 0.25])

        self.ultrasonic_min_range = float(self.get_parameter("ultrasonic_min_range_m").value)
        self.ultrasonic_max_range = float(self.get_parameter("ultrasonic_max_range_m").value)
        self.ultrasonic_timeout_sec = float(self.get_parameter("ultrasonic_timeout_sec").value)

        self._obstacles: List[Tuple[float, float, float]] = []
        self._set_obstacles_from_param()
        self.add_on_set_parameters_callback(self._on_set_parameters)

        # --- Publishers ---
        self.heartbeat_pub = self.create_publisher(Empty, "/base/heartbeat", 10)
        self.stall_pub = self.create_publisher(Bool, "/mower/stall", 10)
        self.ultrasonic_pub = self.create_publisher(UltrasonicArray, "/ultrasonic/ranges", 10)
        self.cmd_vel_safe_pub = self.create_publisher(Twist, "/cmd_vel_gz", 10)

        # --- State ---
        self._robot_x: float = 0.0
        self._robot_y: float = 0.0
        self._robot_yaw: float = 0.0
        self._has_pose: bool = False

        # Fallback: Gazebo gpu_lidar scans (used only when pose unavailable)
        self._last_scans: List[Optional[LaserScan]] = [None] * 6
        self._last_scan_times: List[Optional[Time]] = [None] * 6

        self._last_cmd_vel: Optional[Twist] = None
        self._safety_stop: bool = False
        self._cmd_vel_in_topic = str(self.get_parameter("cmd_vel_in_topic").value)

        # --- Subscriptions ---
        self.create_subscription(Odometry, "/odom/raw", self._on_odom, 10)
        self.create_subscription(Bool, "/safety/stop", self._on_safety_stop, 10)
        self.create_subscription(Twist, self._cmd_vel_in_topic, self._on_cmd_vel, 10)

        scan_topics = [f"/ultrasonic/scan{i}" for i in range(6)]
        for idx, topic in enumerate(scan_topics):
            self.create_subscription(
                LaserScan, topic,
                lambda msg, i=idx: self._on_scan(i, msg),
                10,
            )

        # --- Timers ---
        self.create_timer(0.1,  self._publish_heartbeat)   # 10 Hz
        self.create_timer(1.0,  self._publish_stall)       # 1 Hz
        self.create_timer(0.1,  self._publish_ultrasonic)  # 10 Hz
        self.create_timer(0.05, self._publish_cmd_vel)     # 20 Hz

        self.get_logger().info(
            f"Sim helpers running: heartbeat, stall, pose-based ultrasonic sensing, cmd_vel gating "
            f"(cmd_vel_in_topic='{self._cmd_vel_in_topic}' -> /cmd_vel_gz)"
        )

    def _set_obstacles_from_param(self) -> None:
        raw_obs = list(self.get_parameter("obstacle_positions").value)
        self._obstacles = []
        # Allow clearing with an empty list (no obstacles).
        if len(raw_obs) < 3:
            return
        for i in range(0, len(raw_obs) - 2, 3):
            self._obstacles.append((float(raw_obs[i]), float(raw_obs[i + 1]), float(raw_obs[i + 2])))

    def _on_set_parameters(self, params):
        # Minimal dynamic reconfigure for obstacle list.
        for p in params:
            if p.name == "obstacle_positions":
                # Apply immediately so "no obstacle" tests are truly obstacle-free.
                self._set_obstacles_from_param()
        from rcl_interfaces.msg import SetParametersResult
        return SetParametersResult(successful=True)

    # --- Heartbeat / stall ---
    def _publish_heartbeat(self) -> None:
        self.heartbeat_pub.publish(Empty())

    def _publish_stall(self) -> None:
        msg = Bool()
        msg.data = False
        self.stall_pub.publish(msg)

    # --- Pose from odom ---
    def _on_odom(self, msg: Odometry) -> None:
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self._robot_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self._has_pose = True

    # --- Fallback: Gazebo scan aggregation ---
    def _on_scan(self, index: int, msg: LaserScan) -> None:
        if 0 <= index < 6:
            self._last_scans[index] = msg
            self._last_scan_times[index] = self.get_clock().now()

    # --- Ultrasonic: pose-based (primary) or scan-based (fallback) ---
    def _publish_ultrasonic(self) -> None:
        now = self.get_clock().now()
        ua = UltrasonicArray()
        ua.header.stamp = now.to_msg()
        ua.range_m = [math.nan] * 6
        ua.min_range_m = float(self.ultrasonic_min_range)
        ua.max_range_m = float(self.ultrasonic_max_range)

        if self._has_pose:
            ua.range_m = self._pose_based_ranges()
        else:
            ua.range_m = self._scan_based_ranges(now)

        self.ultrasonic_pub.publish(ua)

    def _pose_based_ranges(self) -> List[float]:
        """Compute sensor ranges using ray-circle intersection against known obstacles."""
        ranges: List[float] = [math.nan] * 6
        if not self._obstacles:
            return ranges

        rx, ry, ryaw = self._robot_x, self._robot_y, self._robot_yaw
        cos_r = math.cos(ryaw)
        sin_r = math.sin(ryaw)

        for i, (lx, ly, lyaw) in enumerate(_SENSOR_POSES):
            # Sensor origin in world frame
            wx = rx + lx * cos_r - ly * sin_r
            wy = ry + lx * sin_r + ly * cos_r
            wyaw = ryaw + lyaw
            dx = math.cos(wyaw)
            dy = math.sin(wyaw)

            best: Optional[float] = None
            for (ox, oy, orad) in self._obstacles:
                dist = _ray_circle_distance(
                    wx, wy, dx, dy,
                    ox, oy, orad,
                    self.ultrasonic_min_range,
                    self.ultrasonic_max_range,
                )
                if dist is not None and (best is None or dist < best):
                    best = dist
            if best is not None:
                ranges[i] = best

        return ranges

    def _scan_based_ranges(self, now: Time) -> List[float]:
        """Fallback: extract min valid range from each Gazebo gpu_lidar scan."""
        ranges: List[float] = [math.nan] * 6
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
                ranges[i] = float(min_valid)
        return ranges

    # --- Cmd_vel gating ---
    def _on_cmd_vel(self, msg: Twist) -> None:
        self._last_cmd_vel = msg

    def _on_safety_stop(self, msg: Bool) -> None:
        self._safety_stop = bool(msg.data)

    def _publish_cmd_vel(self) -> None:
        out = Twist()
        if self._last_cmd_vel is not None and not self._safety_stop:
            out = self._last_cmd_vel
        self.cmd_vel_safe_pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimHelpersNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
