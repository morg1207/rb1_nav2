import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression, FindExecutable

def generate_launch_description():
    package_name = 'localization_server'

    #~~~~~~~~~~~~~~~~~~Declare path~~~~~~~~~~~~~~~
    rviz_file = os.path.join(get_package_share_directory(package_name),'rviz','rviz_config.rviz')
    amcl_file = os.path.join(get_package_share_directory(package_name),'config','amcl_config.yaml')

    #~~~~~~~~~~~~~~~~~~Declare parameters~~~~~~~~~~~~~~~
    # Declara el argumento para el archivo de configuración YAML
    map_file = LaunchConfiguration('map_file')
    arg_map_file = DeclareLaunchArgument(
        'map_file',
        default_value='warehouse_map_sim.yaml',
        description='Path to the map select'
    )
    # Obtener la ruta completa del archivo YAML del mapa
    package_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    map_file_path = PythonExpression(["'",package_path, "/maps","/",map_file, "'"])




    return LaunchDescription([
        arg_map_file,

        #~~~~~~~~~~~~~~~~~~provide map~~~~~~~~~~~~~~~~~~~~~~~~~~
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True}, 
                        {'yaml_filename':map_file_path} 
                       ]),

        #~~~~~~~~~~~~~~~~~~amcl~~~~~~~~~~~~~~~~~~~~~~~~~~
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_file]
            ),
        #~~~~~~~~~~~~~~~~~~lifeclycler_manager~~~~~~~~~~~~~~~~~~~~~~~~~~
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server','amcl']}])

        ]) 