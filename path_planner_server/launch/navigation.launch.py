import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_path_planner = 'path_planner_server'
    package_localization = 'localization_server'
    rviz_file = os.path.join(get_package_share_directory(package_path_planner),'rviz','pathplanning.rviz')
    
    #~~~~~~~~~~~~~~~~~~~~~~~~path config files~~~~~~~~~~~~~~~~~~~~~~~~~~~
    amcl_file = os.path.join(get_package_share_directory(package_localization),'config','amcl_config.yaml')
    controller_yaml = os.path.join(get_package_share_directory(package_path_planner), 'config', 'controller.yaml')
    bt_navigator_yaml = os.path.join(get_package_share_directory(package_path_planner), 'config', 'bt.yaml')
    planner_yaml = os.path.join(get_package_share_directory(package_path_planner), 'config', 'planner_server.yaml')
    recovery_yaml = os.path.join(get_package_share_directory(package_path_planner), 'config', 'recovery.yaml')
    filters_yaml = os.path.join(get_package_share_directory(package_path_planner), 'config', 'filters.yaml')

    #~~~~~~~~~~~~~~~~~~Declare parameters~~~~~~~~~~~~~~~
    # Declara el argumento para el archivo de configuración YAML
    map_file = LaunchConfiguration('map_file')
    arg_map_file = DeclareLaunchArgument(
        'map_file',
        default_value='warehouse_map_sim.yaml',
        description='Path to the map select'
    )

    # Obtener la ruta completa del archivo YAML del mapa
    map_file_path = PythonExpression([
        "'",
        PathJoinSubstitution([
            FindPackageShare(package_localization),
            'maps',
            map_file
        ]),
        "'"
    ])
    # Parámetro de argumento de lanzamiento
    bt_xml_file_arg = DeclareLaunchArgument(
        'default_nav_to_pose_bt_xml',
        default_value=PathJoinSubstitution([
            FindPackageShare('path_planner_server'),
            'config/navigate_w_replanning_and_recovery.xml'
        ]),
        description='Path to the behavior tree XML file for NavigateToPose'
    )

    return LaunchDescription([ 
        bt_xml_file_arg,    
        arg_map_file,

        #~~~~~~~~~~~~~~~~~~provide map~~~~~~~~~~~~~~~~~~~~~~~~~~
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True}, 
                        {'yaml_filename': map_file_path}]
        ),

        #~~~~~~~~~~~~~~~~~~amcl~~~~~~~~~~~~~~~~~~~~~~~~~~
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_file]
        ),
        
        #~~~~~~~~~~~~~~~~~~Controller server ~~~~~~~~~~~~~~~~~~~~~~~~~~
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml]
        ),
    
        #~~~~~~~~~~~~~~~~~~ Path planner server~~~~~~~~~~~~~~~~~~~~~~~~~~
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml]
        ),

        #~~~~~~~~~~~~~~~~~~Behavior server recovery~~~~~~~~~~~~~~~~~~~~~~~~~~
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            parameters=[recovery_yaml],
            output='screen'
        ),
        
        #~~~~~~~~~~~~~~~~~~BT navigator~~~~~~~~~~~~~~~~~~~~~~~~~~
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml, {
                'default_nav_to_pose_bt_xml': LaunchConfiguration('default_nav_to_pose_bt_xml')
            }]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_pathplanner',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names': ['map_server',
                                        'amcl',
                                        'planner_server',
                                        'controller_server',
                                        'behavior_server',
                                        'bt_navigator']}]
        ),

        #~~~~~~~~~~~~~~~~~~rviz2~~~~~~~~~~~~~~~~~~~~~~~~~~      
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_file]
        )
    ])
