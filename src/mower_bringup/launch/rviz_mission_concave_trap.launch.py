"""Concave Trap / Dead-End Test: RViz + mock base.

Robot starts at (0,0), waypoint at (12,0).

U-shaped obstacle (opening toward the robot) is approximated with three circles:

- Left wall:  center ≈ (8.0,  0.8), size (0.5 x 3)  → radius ≈ 0.75 m
- Right wall: center ≈ (8.0, -0.8), size (0.5 x 3)  → radius ≈ 0.75 m
- Back wall:  center ≈ (9.2,  0.0), size (2.0 x 0.5) → radius ≈ 1.0 m

Goals:
- mower can enter the U opening
- BUG/FOLLOW_OBSTACLE cannot find a viable bypass
- DEAD-END DURING DRIVE_ALONG -> BLOCKED_STOP triggers deterministically
- no infinite looping or oscillation between states
- no reliance on safety_stop to end motion
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_mower_base = get_package_share_directory("mower_base")
    pkg_mission = get_package_share_directory("mower_mission")
    pkg_localization = get_package_share_directory("mower_localization")
    pkg_description = get_package_share_directory("mower_description")

    default_mission = os.path.join(
        pkg_mission, "config", "missions", "sim_line_12m.waypoints"
    )
    farm_config = os.path.join(pkg_mission, "config", "farm_origin_sim.yaml")
    rviz_config = os.path.join(pkg_description, "config", "rviz_mission_test.rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "publish_ultrasonic",
                default_value="true",
                description="If true, mock base publishes fake /ultrasonic/ranges.",
            ),
            DeclareLaunchArgument(
                "mission_file",
                default_value=default_mission,
                description="Path to .waypoints mission file (straight 12 m line).",
            ),
            DeclareLaunchArgument(
                "farm_config",
                default_value=farm_config,
                description="Path to farm_origin_sim.yaml for sim missions.",
            ),
            DeclareLaunchArgument(
                "mission_frame_id",
                default_value="odom",
                description="Frame for mission/waypoints (odom). RViz fixed frame.",
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
            # Mock base: concave U-shaped obstacle using three circular approximations.
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
                        "simulate_obstacle_range_m": -1.0,
                        # Left wall (0.5 x 3) → radius ≈ 0.75
                        "virtual_obstacle_center_x": 8.0,
                        "virtual_obstacle_center_y": 0.8,
                        "virtual_obstacle_radius_m": 0.75,
                        # Right wall (0.5 x 3) → radius ≈ 0.75
                        "virtual_obstacle2_center_x": 8.0,
                        "virtual_obstacle2_center_y": -0.8,
                        "virtual_obstacle2_radius_m": 0.75,
                        # Back wall (2 x 0.5) → radius ≈ 1.0
                        "virtual_obstacle3_center_x": 9.2,
                        "virtual_obstacle3_center_y": 0.0,
                        "virtual_obstacle3_radius_m": 1.0,
                        "publish_ultrasonic": LaunchConfiguration("publish_ultrasonic"),
                        "publish_odom": LaunchConfiguration("mock_publish_odom"),
                    }
                ],
            ),
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
                    },
                ],
            ),
            Node(
                package="mower_localization",
                executable="localization_node",
                name="localization",
                parameters=[
                    {
                        "odom_frame_id": "odom",
                        "base_frame_id": "base_link",
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
            ),
        ]
    )

