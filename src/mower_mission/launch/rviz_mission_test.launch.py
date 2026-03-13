import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_mission = get_package_share_directory("mower_mission")
    pkg_base = get_package_share_directory("mower_base")

    rviz_config = os.path.join(pkg_mission, "config", "mission_test.rviz")

    mission_file = LaunchConfiguration("mission_file")
    farm_config = LaunchConfiguration("farm_config")
    mission_frame_id = LaunchConfiguration("mission_frame_id")
    waypoint_tol = LaunchConfiguration("waypoint_tolerance_m")

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_base, "launch", "bringup.launch.py")
        ),
        launch_arguments={
            "publish_ultrasonic": "false",
            "use_ekf": "false",
            "use_gps": "false",
            "use_real_base": "false",
            "base_serial_port": "",
            "publish_odom": "true",
            "mission_file": mission_file,
            "farm_config": farm_config,
            "mission_frame_id": mission_frame_id,
            "simulate_obstacle_range_m": "-1.0",
            "mock_publish_odom": "true",
            "waypoint_tolerance_m": waypoint_tol,
        }.items(),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mission_file",
                default_value=os.path.join(
                    pkg_mission,
                    "config",
                    "missions",
                    "sim_square_5m.waypoints",
                ),
                description="Mission file for RViz-only test (.waypoints, odom frame).",
            ),
            DeclareLaunchArgument(
                "farm_config",
                default_value=os.path.join(
                    pkg_mission,
                    "config",
                    "farm_origin_sim.yaml",
                ),
                description="Farm/mission frame config (odom for sim).",
            ),
            DeclareLaunchArgument(
                "mission_frame_id",
                default_value="odom",
                description="Mission frame (odom for RViz test).",
            ),
            DeclareLaunchArgument(
                "waypoint_tolerance_m",
                default_value="0.5",
                description="Waypoint tolerance (m) for RViz mission test.",
            ),
            bringup_launch,
            rviz_node,
        ]
    )

