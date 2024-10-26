import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression, FindExecutable, PathJoinSubstitution

from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():

    #~~~~~~~~~~~~~~~~~~ PACKAGES ~~~~~~~~~~~~~~~

    package_name = 'localization_server'
    package_maps = 'map_server'

    #~~~~~~~~~~~~~~~~~~ PATHS ~~~~~~~~~~~~~~~

    rviz_file = os.path.join(get_package_share_directory(package_name),'rviz','rviz_config.rviz')
    amcl_file = os.path.join(get_package_share_directory(package_name),'config','amcl_config.yaml')
    amcl_file_real = os.path.join(get_package_share_directory(package_name),'config','amcl_config_real.yaml')

    #~~~~~~~~~~~~~~~~~~ ARGUMENTS ~~~~~~~~~~~~~~~~~~~~

    arg_map_file = DeclareLaunchArgument(
        'map_file',
        choices= ['warehouse_map_sim.yaml','warehouse_map_real.yaml'],
        default_value='warehouse_map_sim.yaml',
        description='Path to the map select'
    )

    arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        choices= ['True', 'False'],
        default_value= 'True',
        description='Path to the map select'
    )

    arg_rviz = DeclareLaunchArgument(
        'rviz',
        choices= ['True', 'False'],
        default_value= 'True',
        description='Path to the map select'
    )
    config_map_file = LaunchConfiguration('map_file')
    config_use_sim_time = LaunchConfiguration('use_sim_time')
    config_rviz = LaunchConfiguration('rviz')
    
    map_file_path = PathJoinSubstitution([get_package_share_directory(package_maps),'maps', config_map_file])


    ##~~~~~~~~~~~~~~~~~~~~~ NODES ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    nav2_map_server = Node(package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': config_use_sim_time},
                {'yaml_filename': map_file_path}
                ]
    )

    #~~~~~~~~~~~~~~~~~~~~~select amcl config file~~~~~~~~~~+
    nav2_amcl_sim = Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                parameters=[amcl_file],
                condition=IfCondition(config_use_sim_time)
    )
    nav2_amcl_real = Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                parameters=[ amcl_file_real],
                condition=UnlessCondition(config_use_sim_time)
    )

    lifecycle_manager = Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_mapper',
                output='screen',
                parameters=[{'use_sim_time': config_use_sim_time},
                            {'autostart': True},
                            {'node_names': ['map_server','amcl']}]
    )
    reinitialize_global_localization = ExecuteProcess(
                cmd=[[
                    FindExecutable(name='ros2'),
                    " service call ",
                    "/reinitialize_global_localization ",
                    "std_srvs/srv/Empty",
            ]],
            shell=True
    )
        
    rviz = Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_file],
                condition=IfCondition(config_rviz)
    )
    return LaunchDescription([
        arg_map_file,
        arg_use_sim_time,
        arg_rviz,
        nav2_map_server,
        nav2_amcl_sim,
        nav2_amcl_real,
        lifecycle_manager,
        rviz,
        reinitialize_global_localization
        ]) 