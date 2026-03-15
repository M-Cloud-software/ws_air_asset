from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('tarp_detection')
    detection_cfg = os.path.join(pkg, 'config', 'detection_params.yaml')
    monitor_cfg   = os.path.join(pkg, 'config', 'monitor_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('image_path',  default_value=''),
        DeclareLaunchArgument('start_lat',   default_value='37.7749'),
        DeclareLaunchArgument('start_lon',   default_value='-122.4194'),
        DeclareLaunchArgument('altitude_m',  default_value='30.0'),
        DeclareLaunchArgument('speed_mps',   default_value='3.0'),
        DeclareLaunchArgument('fps',         default_value='10.0'),

        Node(
            package='tarp_detection',
            executable='sitl_publisher',
            name='sitl_publisher',
            output='screen',
            parameters=[{
                'image_path': LaunchConfiguration('image_path'),
                'start_lat':  LaunchConfiguration('start_lat'),
                'start_lon':  LaunchConfiguration('start_lon'),
                'altitude_m': LaunchConfiguration('altitude_m'),
                'speed_mps':  LaunchConfiguration('speed_mps'),
                'fps':        LaunchConfiguration('fps'),
            }],
        ),

        Node(
            package='tarp_detection',
            executable='tarp_detection_node',
            name='tarp_detection',
            output='screen',
            parameters=[detection_cfg],
        ),

        Node(
            package='tarp_detection',
            executable='pipeline_monitor',
            name='pipeline_monitor',
            output='screen',
            parameters=[monitor_cfg],
        ),
    ])
