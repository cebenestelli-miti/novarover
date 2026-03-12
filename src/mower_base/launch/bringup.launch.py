import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, EqualsSubstitution, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_localization = get_package_share_directory('mower_localization')
    pkg_mission = get_package_share_directory('mower_mission')
    default_mission = os.path.join(pkg_mission, 'config', 'missions', 'farm_triangle.wgs84')
    farm_config = os.path.join(pkg_mission, 'config', 'farm_origin.yaml')
    ekf_config = os.path.join(pkg_localization, 'config', 'ekf.yaml')
    ekf_gps_config = os.path.join(pkg_localization, 'config', 'ekf_gps.yaml')
    navsat_config = os.path.join(pkg_localization, 'config', 'navsat_transform.yaml')

    use_ekf = LaunchConfiguration('use_ekf')
    use_gps = LaunchConfiguration('use_gps')
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
        DeclareLaunchArgument('mission_file', default_value=default_mission,
            description='Path to .waypoints or .wgs84 mission file. Empty to use waypoints param only.'),
        DeclareLaunchArgument('farm_config', default_value=farm_config,
            description='Path to farm_origin.yaml (farm_origin_lat/lon/alt, geofence, GNSS gates).'),
        DeclareLaunchArgument('mission_frame_id', default_value='map',
            description='Frame for mission/waypoints (odom or map). Use odom for Gazebo sim.'),
        DeclareLaunchArgument('simulate_obstacle_range_m', default_value='-1.0',
            description='Mock only: if > 0, front ultrasonics report this range (m) to test go-around (blocked-skip). -1 = off.'),
        DeclareLaunchArgument('mock_publish_odom', default_value='true',
            description='Mock base: set false when Gazebo bridge provides /odom/raw to avoid dual-publisher conflict.'),
    DeclareLaunchArgument('waypoint_tolerance_m', default_value='0.2',
        description='Waypoint reach tolerance (meters) for waypoint_follower.'),
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
                'simulate_obstacle_range_m': LaunchConfiguration('simulate_obstacle_range_m'),
                'publish_ultrasonic': LaunchConfiguration('publish_ultrasonic'),
                'publish_odom': LaunchConfiguration('mock_publish_odom'),
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
                'publish_ultrasonic': LaunchConfiguration('publish_ultrasonic'),
                'serial_port': LaunchConfiguration('base_serial_port'),
                'publish_odom': LaunchConfiguration('publish_odom'),
            }],
        ),
        Node(
            package='ros_mower_core',
            executable='safety_manager_node',
            name='safety_manager',
            parameters=[{
                'require_estop_release': True,
                # Increased from 0.5s: WSL2 Gazebo can pause physics briefly during
                # rapid turns, causing sim_helpers heartbeat gaps up to ~3s.
                'base_timeout_sec': 5.0,
                'heartbeat_startup_grace_sec': 8.0,
            }],
        ),
        Node(
            package='mower_obstacles',
            executable='ultrasonic_guard_node',
            name='ultrasonic_guard',
            parameters=[{'stop_dist_m': 0.30, 'blocked_dist_m': 1.20, 'caution_dist_m': 2.0}],
        ),
        Node(
            package='mower_mission',
            executable='mission_manager_node',
            name='mission_manager',
            parameters=[
                LaunchConfiguration('farm_config'),
                {
                    'mission_file': LaunchConfiguration('mission_file'),
                    'mission_frame_id': LaunchConfiguration('mission_frame_id'),
                },
            ],
        ),
        Node(
            package='mower_mission',
            executable='waypoint_follower_node',
            name='waypoint_follower',
            parameters=[
                LaunchConfiguration('farm_config'),
                {
                    'max_linear_speed': 0.5,
                    'mission_file': LaunchConfiguration('mission_file'),
                    'mission_frame_id': LaunchConfiguration('mission_frame_id'),
                    'waypoint_tolerance_m': LaunchConfiguration('waypoint_tolerance_m'),
                },
            ],
            # Isolate navigation command stream to avoid interference from other /cmd_vel publishers
            # (teleop, stale nodes, etc.). Gazebo sim_helpers can be configured to listen to this.
            remappings=[('/cmd_vel', '/cmd_vel_nav')],
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
