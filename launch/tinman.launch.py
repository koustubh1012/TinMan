import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess

def generate_launch_description():
    
    tinman_cmd = Node(
        package='tinman',
        executable='tinman_exec',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    map_path = os.path.join(get_package_share_directory('tinman'), 'maps', 'cafeteria.yaml')
    
    turtlebot3_navigation2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("turtlebot3_navigation2"),
             "/launch", "/navigation2.launch.py"]
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'map': map_path,
            'rvizconfig': os.path.join(
                get_package_share_directory('tinman'),
                'rviz', 'cafeteria.rviz'
            ),
        }.items()
    )
    
    static_transform_publisher_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    
    
    ld = LaunchDescription()
    ld.add_action(turtlebot3_navigation2_cmd)
    ld.add_action(static_transform_publisher_cmd)
    ld.add_action(tinman_cmd)
    return ld