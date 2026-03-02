# Launch only safety_manager + ultrasonic_guard (no mock).
# Use for heartbeat-timeout test: no /base/heartbeat, so safety asserts after grace+timeout.
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_mower_core',
            executable='safety_manager_node',
            name='safety_manager',
            parameters=[{'require_estop_release': False}],
        ),
        Node(
            package='mower_obstacles',
            executable='ultrasonic_guard_node',
            name='ultrasonic_guard',
            parameters=[{'stop_dist_m': 0.60}],
        ),
    ])
