from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch_ros.actions import Node, RosTimer
from launch.event_handlers import OnExecutionComplete, OnProcessStart
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
     


    navigation = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("path_planner_server"), '/launch', '/navigation.launch.py'])
            )
    
    servers = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("rb1_shelf_tools"), '/launch', '/servers.launch.py'])
            )
    return LaunchDescription([
        navigation,
        RosTimer(period=5.0, actions=[servers]),
        Node(
            package='rb1_shelf_tools',
            executable='bt_selector_behavior_tree',
            name='bt_selector_behavior_tree',
            output='screen',
        ),

    ])