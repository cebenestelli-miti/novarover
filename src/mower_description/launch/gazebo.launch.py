import os
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix


def generate_launch_description():
    pkg_mower = get_package_share_directory('mower_description')
    pkg_prefix = get_package_prefix('mower_description')
    pkg_gz_sim = get_package_share_directory('ros_gz_sim')
    world_path = os.path.join(pkg_mower, 'worlds', 'mower_empty.sdf')
    bridge_config = os.path.join(pkg_mower, 'config', 'bridge.yaml')
    resource_path = os.path.dirname(pkg_mower)
    sim_helpers_script = os.path.join(pkg_prefix, 'lib', 'mower_description', 'sim_helpers_node.py')

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=world_path, description='Full path to world SDF file'),
        DeclareLaunchArgument('cmd_vel_in_topic', default_value='/cmd_vel',
                              description='ROS topic for sim_helpers to consume before gating to /cmd_vel_gz'),
        # So Gazebo resolves package://mower_description/...
        AppendEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=resource_path),
        # Start Gazebo with the world (world includes mower model)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': [LaunchConfiguration('world'), ' -r']}.items(),
        ),
        # Bridge ROS <-> Gazebo (after short delay so sim is up)
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    name='ros_gz_bridge',
                    arguments=['--ros-args', '-p', 'config_file:=' + bridge_config],
                    output='screen',
                ),
                # Sim topic contract: /base/heartbeat (10 Hz), /mower/stall (1 Hz, false)
                ExecuteProcess(
                    cmd=[
                        'python3', sim_helpers_script,
                        '--ros-args', '-p', ['cmd_vel_in_topic:=', LaunchConfiguration('cmd_vel_in_topic')]
                    ],
                    name='sim_helpers_node',
                    output='screen',
                ),
            ],
        ),
    ])
