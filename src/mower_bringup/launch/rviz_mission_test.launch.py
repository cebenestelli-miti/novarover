"""RViz-only mission test: mock base + safety + mission + localization + RViz (no Gazebo)."""

import os
import math
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_mower_base = get_package_share_directory("mower_base")
    pkg_mission = get_package_share_directory("mower_mission")
    pkg_localization = get_package_share_directory("mower_localization")
    pkg_description = get_package_share_directory("mower_description")

    default_mission = os.path.join(
        pkg_mission, "config", "missions", "sim_line_10m.waypoints"
    )
    farm_config = os.path.join(pkg_mission, "config", "farm_origin_sim.yaml")
    rviz_config = os.path.join(pkg_description, "config", "rviz_mission_test.rviz")
    mission_file = LaunchConfiguration("mission_file")
    mission_frame_id = LaunchConfiguration("mission_frame_id")

    def _make_mock_base_node(context):
        """Create mock_base_interface_node, optionally spawning near first WGS84 waypoint."""
        mf = mission_file.perform(context)
        frame = mission_frame_id.perform(context)
        auto_flag = LaunchConfiguration("auto_spawn_near_first_wp").perform(context).lower() in (
            "true",
            "1",
            "yes",
        )

        initial_x = 0.0
        initial_y = 0.0
        initial_yaw = 0.0

        # Only attempt auto spawn when explicitly requested, using real map frame
        # and a .wgs84 mission file.
        if auto_flag and frame == "map" and mf.endswith(".wgs84"):
            try:
                farm_cfg_path = LaunchConfiguration("farm_config").perform(context)
                with open(farm_cfg_path, "r") as f:
                    cfg = yaml.safe_load(f) or {}
                mm = cfg.get("mission_manager", {}).get("ros__parameters", {})
                lat0 = float(mm.get("farm_origin_lat", 0.0))
                lon0 = float(mm.get("farm_origin_lon", 0.0))

                first_lat = None
                first_lon = None
                with open(mf, "r") as f:
                    for line in f:
                        s = line.strip()
                        if not s or s.startswith("#"):
                            continue
                        parts = s.split()
                        if len(parts) < 2:
                            continue
                        try:
                            first_lat = float(parts[0])
                            first_lon = float(parts[1])
                            break
                        except ValueError:
                            continue

                if first_lat is not None and first_lon is not None:
                    # Match mission_loader.cpp WGS84 -> ENU approximation.
                    DEG2RAD = math.pi / 180.0
                    METERS_PER_DEG_LAT = 111320.0
                    lat0_rad = lat0 * DEG2RAD
                    north_m = (first_lat - lat0) * METERS_PER_DEG_LAT
                    east_m = (first_lon - lon0) * METERS_PER_DEG_LAT * math.cos(lat0_rad)

                    # Spawn a few meters "before" the first waypoint along -x for visibility.
                    initial_x = east_m - 5.0
                    initial_y = north_m
                    dx = east_m - initial_x
                    dy = north_m - initial_y
                    initial_yaw = math.atan2(dy, dx)
            except Exception:
                initial_x = 0.0
                initial_y = 0.0
                initial_yaw = 0.0

        return [
            Node(
                package="mower_base",
                executable="mock_base_interface_node",
                name="mock_base_interface",
                parameters=[
                    {
                        "publish_rate_hz": 30.0,
                        "ultrasonic_clear_range_m": 2.0,
                        "ultrasonic_min_range_m": 0.02,
                        "ultrasonic_max_range_m": 4.0,
                        "simulate_obstacle_range_m": LaunchConfiguration(
                            "simulate_obstacle_range_m"
                        ),
                        # Three-obstacle straight-line test (0,0 -> 10,0):
                        # 1: left offset, 2: right offset, 3: left offset again.
                        "virtual_obstacle_center_x": 3.0,
                        "virtual_obstacle_center_y": 0.7,
                        "virtual_obstacle_radius_m": 0.5,
                        "virtual_obstacle2_center_x": 6.0,
                        "virtual_obstacle2_center_y": -0.7,
                        "virtual_obstacle2_radius_m": 0.5,
                        "virtual_obstacle3_center_x": 8.5,
                        "virtual_obstacle3_center_y": 0.7,
                        "virtual_obstacle3_radius_m": 0.5,
                        "publish_ultrasonic": LaunchConfiguration("publish_ultrasonic"),
                        "publish_odom": LaunchConfiguration("mock_publish_odom"),
                        "odom_frame_id": mission_frame_id,
                        "initial_x": initial_x,
                        "initial_y": initial_y,
                        "initial_yaw_rad": initial_yaw,
                    }
                ],
            )
        ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "publish_ultrasonic",
                default_value="true",
                description="If true, mock base publishes fake /ultrasonic/ranges.",
            ),
            DeclareLaunchArgument(
                "simulate_obstacle_range_m",
                default_value="-1.0",
                description=(
                    "Mock only: if > 0, front ultrasonics report this range (m) "
                    "to test go-around (blocked-skip). -1 = off."
                ),
            ),
            DeclareLaunchArgument(
                "mission_file",
                default_value=default_mission,
                description="Path to .waypoints mission file used for RViz test.",
            ),
            DeclareLaunchArgument(
                "farm_config",
                default_value=farm_config,
                description="Path to farm_origin_sim.yaml for sim missions.",
            ),
            DeclareLaunchArgument(
                "mission_frame_id",
                default_value="odom",
                description="Frame for mission/waypoints (odom or map). RViz fixed frame.",
            ),
            DeclareLaunchArgument(
                "waypoint_tolerance_m",
                default_value="0.5",
                description="Waypoint reach tolerance (meters) for waypoint_follower.",
            ),
            DeclareLaunchArgument(
                "mock_publish_odom",
                default_value="true",
                description="Mock base: set false when another source provides /odom/raw.",
            ),
            DeclareLaunchArgument(
                "auto_spawn_near_first_wp",
                default_value="false",
                description=(
                    "If true and using a .wgs84 mission with mission_frame_id:=map, "
                    "spawn the mock base near the first waypoint for easier RViz viewing."
                ),
            ),
            # Mock base (simulated odometry + ultrasonics), optionally auto-spawned
            # near the first WGS84 waypoint when requested.
            OpaqueFunction(function=_make_mock_base_node),
            # Safety manager + ultrasonic guard
            Node(
                package="ros_mower_core",
                executable="safety_manager_node",
                name="safety_manager",
                parameters=[
                    {
                        "require_estop_release": True,
                        "base_timeout_sec": 5.0,
                        "heartbeat_startup_grace_sec": 8.0,
                    }
                ],
            ),
            Node(
                package="mower_obstacles",
                executable="ultrasonic_guard_node",
                name="ultrasonic_guard",
                parameters=[
                    {
                        "stop_dist_m": 0.30,
                        "blocked_dist_m": 1.20,
                        "caution_dist_m": 2.0,
                    }
                ],
            ),
            # Mission manager + waypoint follower (publishes /cmd_vel)
            Node(
                package="mower_mission",
                executable="mission_manager_node",
                name="mission_manager",
                parameters=[
                    LaunchConfiguration("farm_config"),
                    {
                        "mission_file": LaunchConfiguration("mission_file"),
                        "mission_frame_id": LaunchConfiguration("mission_frame_id"),
                    },
                ],
            ),
            Node(
                package="mower_mission",
                executable="waypoint_follower_node",
                name="waypoint_follower",
                parameters=[
                    LaunchConfiguration("farm_config"),
                    {
                        "max_linear_speed": 0.5,
                        "mission_file": LaunchConfiguration("mission_file"),
                        "mission_frame_id": LaunchConfiguration("mission_frame_id"),
                        "waypoint_tolerance_m": LaunchConfiguration(
                            "waypoint_tolerance_m"
                        ),
                        # Match mock base three-obstacle layout (visible in RViz).
                        "debug_obstacle_center_x": 3.0,
                        "debug_obstacle_center_y": 0.7,
                        "debug_obstacle_radius_m": 0.5,
                        "debug_obstacle2_center_x": 6.0,
                        "debug_obstacle2_center_y": -0.7,
                        "debug_obstacle2_radius_m": 0.5,
                        "debug_obstacle3_center_x": 8.5,
                        "debug_obstacle3_center_y": 0.7,
                        "debug_obstacle3_radius_m": 0.5,
                    },
                ],
            ),
            # Simple odom forward + TF odom->base_link
            Node(
                package="mower_localization",
                executable="localization_node",
                name="localization",
                parameters=[
                    {
                        # Keep localization and mission/waypoints in the same frame so that
                        # RViz visualization and mission_frame_id stay aligned. This allows
                        # using either "odom" (sim) or "map" (real farm origin) consistently.
                        "odom_frame_id": LaunchConfiguration("mission_frame_id"),
                        "base_frame_id": "base_link",
                    }
                ],
            ),
            # RViz with mission visualization config
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
            ),
        ]
    )

