import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, EqualsSubstitution, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_localization = get_package_share_directory('mower_localization')
    ekf_config = os.path.join(pkg_localization, 'config', 'ekf.yaml')
    ekf_gps_config = os.path.join(pkg_localization, 'config', 'ekf_gps.yaml')
    navsat_config = os.path.join(pkg_localization, 'config', 'navsat_transform.yaml')

    use_ekf = LaunchConfiguration('use_ekf', default='false')
    use_gps = LaunchConfiguration('use_gps', default='false')
    # Condition: use_ekf and use_gps both true (for EKF+GPS and navsat)
    use_ekf_gps = PythonExpression(["'true' if '", use_ekf, "' == 'true' and '", use_gps, "' == 'true' else 'false'"])
    # Condition: use_ekf true and use_gps false (EKF without GPS)
    use_ekf_no_gps = PythonExpression(["'true' if '", use_ekf, "' == 'true' and '", use_gps, "' != 'true' else 'false'"])

    return LaunchDescription([
        DeclareLaunchArgument('publish_ultrasonic', default_value='true',
            description='If true, mock base publishes fake /ultrasonic/ranges. Set false for physical deployment.'),
        DeclareLaunchArgument('use_ekf', default_value='false',
            description='If true, run EKF to fuse /odom/raw + /imu/data. Requires /imu/data (e.g. BNO085).'),
        DeclareLaunchArgument('use_gps', default_value='false',
            description='If true (and use_ekf true), fuse GPS via navsat_transform. Requires /gps/fix (e.g. NEO-M9N).'),
        DeclareLaunchArgument('use_real_base', default_value='false',
            description='If true, run real base interface (Arduino bridge) instead of mock. Set serial_port for hardware.'),
        DeclareLaunchArgument('base_serial_port', default_value='',
            description='Serial port for real base (e.g. /dev/ttyUSB0). Empty = stub mode (heartbeat + zero odom).'),
        DeclareLaunchArgument('publish_odom', default_value='true',
            description='When use_real_base: if false, real base does not publish /odom/raw (e.g. when Gazebo bridge provides it).'),
        # Mock base: for development/testing without hardware
        Node(
            package='mower_base',
            executable='mock_base_interface_node',
            name='mock_base_interface',
            condition=UnlessCondition(EqualsSubstitution(LaunchConfiguration('use_real_base'), 'true')),
            parameters=[{
                'publish_rate_hz': 10.0,
                'ultrasonic_clear_range_m': 2.0,
                'ultrasonic_min_range_m': 0.02,
                'ultrasonic_max_range_m': 4.0,
                'publish_ultrasonic': LaunchConfiguration('publish_ultrasonic'),
            }],
        ),
        # Real base (Arduino bridge): for physical deployment or Gazebo (stub with publish_odom:=false)
        Node(
            package='mower_base',
            executable='real_base_interface_node',
            name='real_base_interface',
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration('use_real_base'), 'true')),
            parameters=[{
                'publish_rate_hz': 10.0,
                'publish_ultrasonic': True,
                'serial_port': LaunchConfiguration('base_serial_port', default_value=''),
                'publish_odom': LaunchConfiguration('publish_odom', default_value='true'),
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
        # EKF without GPS: /odom/raw + /imu/data
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            condition=IfCondition(use_ekf_no_gps),
            parameters=[ekf_config],
            remappings=[('odometry/filtered', '/odom')],
        ),
        # EKF with GPS: /odom/raw + /imu/data + /odometry/gps
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            condition=IfCondition(use_ekf_gps),
            parameters=[ekf_gps_config],
            remappings=[('odometry/filtered', '/odom')],
        ),
        # NavSat: /gps/fix + /imu/data + /odom -> /odometry/gps (for EKF when use_gps)
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            condition=IfCondition(use_ekf_gps),
            parameters=[navsat_config],
            remappings=[
                ('odometry/filtered', '/odom'),
                ('imu/data', '/imu/data'),
                ('gps/fix', '/gps/fix'),
                ('odometry/gps', '/odometry/gps'),
            ],
        ),
    ])
