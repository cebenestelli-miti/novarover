import os
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_mower = get_package_share_directory('mower_description')
    pkg_gz_sim = get_package_share_directory('ros_gz_sim')
    world_path = os.path.join(pkg_mower, 'worlds', 'mower_empty.sdf')
    bridge_config = os.path.join(pkg_mower, 'config', 'bridge.yaml')
    resource_path = os.path.dirname(pkg_mower)

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=world_path, description='Full path to world SDF file'),
        # So Gazebo resolves package://mower_description/...
        AppendEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=resource_path),
        # Start Gazebo with the world (world includes mower model)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': LaunchConfiguration('world')}.items(),
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
            ],
        ),
    ])
