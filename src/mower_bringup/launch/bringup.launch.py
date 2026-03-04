"""Bring up base (mock/real), safety, ultrasonic_guard, mission, waypoint_follower, localization."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_mower_base = get_package_share_directory('mower_base')
    bringup_launch = os.path.join(pkg_mower_base, 'launch', 'bringup.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('publish_ultrasonic', default_value='true'),
        DeclareLaunchArgument('use_ekf', default_value='false'),
        DeclareLaunchArgument('use_gps', default_value='false'),
        DeclareLaunchArgument('use_real_base', default_value='false'),
        DeclareLaunchArgument('base_serial_port', default_value=''),
        DeclareLaunchArgument('publish_odom', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch),
            launch_arguments={
                'publish_ultrasonic': LaunchConfiguration('publish_ultrasonic', default='true'),
                'use_ekf': LaunchConfiguration('use_ekf', default='false'),
                'use_gps': LaunchConfiguration('use_gps', default='false'),
                'use_real_base': LaunchConfiguration('use_real_base', default='false'),
                'base_serial_port': LaunchConfiguration('base_serial_port', default=''),
                'publish_odom': LaunchConfiguration('publish_odom', default='true'),
            }.items(),
        ),
    ])
