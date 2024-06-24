import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = 'path_planner_server'
    rviz_file = os.path.join(get_package_share_directory(package_name),'rviz','pathplanning.rviz')
    #~~~~~~~~~~~~~~~~~~~~~~~~path config files~~~~~~~~~~~~~~~~~~~~~~~~~~~+
    controller_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'controller.yaml')
    bt_navigator_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'bt.yaml')
    planner_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'planner_server.yaml')
    recovery_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'recovery.yaml')
    filters_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'filters.yaml')

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
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml]),
    

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml]),
            
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            parameters=[recovery_yaml],
            output='screen'),

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
                        {'use_sim_time': True},
                        {'node_names': ['planner_server',
                                        'controller_server',
                                        'behavior_server',
                                        'bt_navigator'
                                        ]}]),

        #~~~~~~~~~~~~~~~~~~rviz2~~~~~~~~~~~~~~~~~~~~~~~~~~      
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_file],
        ),
    ])
