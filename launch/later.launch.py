import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.actions import TimerAction

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = os.path.join(get_package_share_directory('tinman'), 'worlds', 'cafeteria.world')

    bin_model_dir = os.path.join(get_package_share_directory('tinman'), 'models','green_can','model.sdf')

    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    map_dir = os.path.join(get_package_share_directory('tinman'), 'maps', 'cafeteria.yaml')

    
    nav2_man = get_package_share_directory('turtlebot3_manipulation_navigation2')
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_man, 'launch', 'navigation2.launch.py')),
        launch_arguments={
            'map_yaml_file': map_dir,
            'start_rviz': 'True',
            'params_file': os.path.join(nav2_man, 'param', 'turtlebot3_use_sim_time.yaml'),
            'autostart': 'True'
        }.items()
    )  
    
    initial_pose_pub = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '-1', '/initialpose', 'geometry_msgs/msg/PoseWithCovarianceStamped',
            '{header: {frame_id: "map"}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}'
        ],
        shell=True
    )
    
    static_transform_publisher_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    ) 

    ld=LaunchDescription()
    # ld.add_action(nav_launch)
    ld.add_action(initial_pose_pub)
    ld.add_action(static_transform_publisher_cmd)

    return ld
