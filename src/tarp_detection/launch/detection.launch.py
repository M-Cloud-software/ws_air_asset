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
        DeclareLaunchArgument('image_topic',
            default_value='/camera_feed'),
        DeclareLaunchArgument('global_pos_topic',
            default_value='/fmu/out/vehicle_global_position'),
        DeclareLaunchArgument('local_pos_topic',
            default_value='/fmu/out/vehicle_local_position'),

        Node(
            package='tarp_detection',
            executable='tarp_detection_node',
            name='tarp_detection',
            output='screen',
            parameters=[detection_cfg, {
                'image_topic':      LaunchConfiguration('image_topic'),
                'global_pos_topic': LaunchConfiguration('global_pos_topic'),
                'local_pos_topic':  LaunchConfiguration('local_pos_topic'),
            }],
        ),

        Node(
            package='tarp_detection',
            executable='pipeline_monitor',
            name='pipeline_monitor',
            output='screen',
            parameters=[monitor_cfg],
        ),
    ])
