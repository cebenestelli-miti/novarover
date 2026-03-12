"""RViz-only mission test: mock base + safety + mission + localization + RViz (no Gazebo)."""

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
        pkg_mission, "config", "missions", "sim_line_10m.waypoints"
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
                default_value="0.2",
                description="Waypoint reach tolerance (meters) for waypoint_follower.",
            ),
            DeclareLaunchArgument(
                "mock_publish_odom",
                default_value="true",
                description="Mock base: set false when another source provides /odom/raw.",
            ),
            # Mock base (simulated odometry + ultrasonics)
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
                        # Virtual 1x1 m box at 5 m along +X (~circle radius 0.5 m)
                        "virtual_obstacle_center_x": 5.0,
                        "virtual_obstacle_center_y": 0.0,
                        "virtual_obstacle_radius_m": 0.5,
                        "publish_ultrasonic": LaunchConfiguration("publish_ultrasonic"),
                        "publish_odom": LaunchConfiguration("mock_publish_odom"),
                    }
                ],
            ),
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
                        # Match virtual obstacle used by mock base so Bug behavior vs. box is visible.
                        "debug_obstacle_center_x": 5.0,
                        "debug_obstacle_center_y": 0.0,
                        "debug_obstacle_radius_m": 0.5,
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
                        "odom_frame_id": "odom",
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

