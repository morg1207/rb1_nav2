import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    package_name = 'cartographer_slam'
    cartographer_config_dir = os.path.join(get_package_share_directory(package_name), 'config')
    configuration_basename = 'cartographer.lua'
    rviz_file = os.path.join(get_package_share_directory(package_name), 'rviz', 'rviz_config.rviz')

    use_sim_time_value = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true', description='Parameter to use simulation time or real robot time')

    resolution_map_value = LaunchConfiguration('resolution_map')
    resolution_map_arg = DeclareLaunchArgument('resolution_map', default_value='0.05', description='Map resolution')

    return LaunchDescription([
        # Argumentos
        use_sim_time_arg,
        resolution_map_arg,

        # SLAM Algorithm
        Node(
            package='cartographer_ros', 
            executable='cartographer_node', 
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', configuration_basename
            ]
        ),

        # Publish Map
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=[
                '-resolution', resolution_map_value,
                '-publish_period_sec', '1.0'
            ]
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_file]
        ),
    ])
