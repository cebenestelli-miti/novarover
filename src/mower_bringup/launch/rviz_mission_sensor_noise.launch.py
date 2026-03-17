"""Sensor noise test: RViz + mock base with simulated ultrasonic noise.

Runs a normal mission (random_field_20m) with no physical obstacles. The mock base
injects realistic sensor noise: Gaussian range noise, occasional false obstacle
readings, and short dropouts. Verifies that:

- Robot does not stop or oscillate excessively
- Occasional noise does not cause permanent FOLLOW_OBSTACLE state
- Robot continues progressing through waypoints
- Mission completes successfully

Noise is implemented in the simulation layer only (mock_base_interface_node);
core navigation logic is unchanged.
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

    mission = os.path.join(
        pkg_mission, "config", "missions", "random_field_20m.waypoints"
    )
    farm_config = os.path.join(pkg_mission, "config", "farm_origin_sim.yaml")
    rviz_config = os.path.join(
        pkg_description, "config", "rviz_mission_random_obstacles.rviz"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mission_file",
                default_value=mission,
                description="Path to .waypoints mission file.",
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
                description="Waypoint reach tolerance (meters).",
            ),
            DeclareLaunchArgument(
                "sensor_noise_test",
                default_value="true",
                description="Enable ultrasonic noise/dropout injection in mock base.",
            ),
            DeclareLaunchArgument(
                "ultrasonic_noise_std_m",
                default_value="0.05",
                description="Gaussian noise std dev (m) on ultrasonic ranges.",
            ),
            DeclareLaunchArgument(
                "ultrasonic_false_obstacle_prob",
                default_value="0.02",
                description="Per-tick probability of a false short-range reading on one front sensor.",
            ),
            DeclareLaunchArgument(
                "ultrasonic_false_obstacle_range_m",
                default_value="0.4",
                description="Range (m) reported when injecting a false obstacle.",
            ),
            DeclareLaunchArgument(
                "ultrasonic_dropout_prob",
                default_value="0.01",
                description="Per-tick probability of starting a short dropout (no publish).",
            ),
            DeclareLaunchArgument(
                "ultrasonic_dropout_max_ticks",
                default_value="2",
                description="Max number of consecutive ticks to drop when dropout triggers.",
            ),
            # Mock base: no virtual obstacles; sensor noise test enabled
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
                        "virtual_obstacle_radius_m": -1.0,
                        "virtual_obstacle2_radius_m": -1.0,
                        "virtual_obstacle3_radius_m": 0.0,
                        "publish_ultrasonic": True,
                        "publish_odom": True,
                        "sensor_noise_test": LaunchConfiguration("sensor_noise_test"),
                        "ultrasonic_noise_std_m": LaunchConfiguration(
                            "ultrasonic_noise_std_m"
                        ),
                        "ultrasonic_false_obstacle_prob": LaunchConfiguration(
                            "ultrasonic_false_obstacle_prob"
                        ),
                        "ultrasonic_false_obstacle_range_m": LaunchConfiguration(
                            "ultrasonic_false_obstacle_range_m"
                        ),
                        "ultrasonic_dropout_prob": LaunchConfiguration(
                            "ultrasonic_dropout_prob"
                        ),
                        "ultrasonic_dropout_max_ticks": LaunchConfiguration(
                            "ultrasonic_dropout_max_ticks"
                        ),
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
            # Mission manager + waypoint follower
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
            # Localization + RViz
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
