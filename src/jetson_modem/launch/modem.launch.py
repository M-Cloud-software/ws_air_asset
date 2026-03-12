from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    detection_cfg = os.path.join(
        get_package_share_directory('tarp_detection'),
        'config', 'detection_params.yaml'
    )
    modem_cfg = os.path.join(
        get_package_share_directory('jetson_modem'),
        'config', 'modem_params.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument('server_ip',
            default_value='192.168.1.100',
            description='Ground station laptop IP address'),

        Node(
            package='tarp_detection',
            executable='tarp_detection_node',
            name='tarp_detection',
            output='screen',
            parameters=[detection_cfg],
        ),

        Node(
            package='jetson_modem',
            executable='jetson_modem_node',
            name='jetson_modem',
            output='screen',
            parameters=[modem_cfg, {
                'server_ip': LaunchConfiguration('server_ip'),
            }],
        ),
    ])
