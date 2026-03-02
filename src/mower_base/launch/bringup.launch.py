import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, EqualsSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_ekf = LaunchConfiguration('use_ekf', default='false')
    ekf_config = os.path.join(get_package_share_directory('mower_localization'), 'config', 'ekf.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('publish_ultrasonic', default_value='true',
            description='If true, mock base publishes fake /ultrasonic/ranges. Set false for physical deployment.'),
        DeclareLaunchArgument('use_ekf', default_value='false',
            description='If true, run EKF (robot_localization) to fuse /odom/raw + /imu/data. Requires /imu/data (e.g. BNO085).'),
        Node(
            package='mower_base',
            executable='mock_base_interface_node',
            name='mock_base_interface',
            parameters=[{
                'publish_rate_hz': 10.0,
                'ultrasonic_clear_range_m': 2.0,
                'ultrasonic_min_range_m': 0.02,
                'ultrasonic_max_range_m': 4.0,
                'publish_ultrasonic': LaunchConfiguration('publish_ultrasonic'),
            }],
        ),
        Node(
            package='ros_mower_core',
            executable='safety_manager_node',
            name='safety_manager',
            parameters=[{'require_estop_release': True}],
        ),
        Node(
            package='mower_obstacles',
            executable='ultrasonic_guard_node',
            name='ultrasonic_guard',
            parameters=[{'stop_dist_m': 0.60}],
        ),
        Node(
            package='mower_mission',
            executable='mission_manager_node',
            name='mission_manager',
        ),
        Node(
            package='mower_mission',
            executable='waypoint_follower_node',
            name='waypoint_follower',
            parameters=[{
                'waypoint_tolerance_m': 0.4,
                'max_linear_speed': 0.5,
                'max_angular_speed': 0.8,
                # waypoints parameter is left to the node default (empty)
                # and should be overridden via a YAML params file or CLI.
            }],
        ),
        # Simple odom forward (no IMU/GPS fusion). Used when use_ekf is false.
        Node(
            package='mower_localization',
            executable='localization_node',
            name='localization',
            condition=UnlessCondition(EqualsSubstitution(LaunchConfiguration('use_ekf'), 'true')),
            parameters=[{
                'odom_frame_id': 'odom',
                'base_frame_id': 'base_link',
            }],
        ),
        # EKF fusion: /odom/raw + /imu/data (BNO085). Use when IMU is connected; add GPS (NEO-M9N) later via navsat_transform.
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration('use_ekf'), 'true')),
            parameters=[ekf_config],
            remappings=[('odometry/filtered', '/odom')],
        ),
    ])
