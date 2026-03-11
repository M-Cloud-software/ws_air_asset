"""
detection.launch.py  –  production launch (PX4 uXRCE-DDS)

ros2 launch tarp_detection detection.launch.py
ros2 launch tarp_detection detection.launch.py \
    image_topic:=/camera/image_raw
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory("tarp_detection"),
        "config", "detection_params.yaml",
    )

    return LaunchDescription([
        DeclareLaunchArgument("image_topic",
            default_value="/camera_feed"),
        DeclareLaunchArgument("global_pos_topic",
            default_value="/fmu/out/vehicle_global_position"),
        DeclareLaunchArgument("local_pos_topic",
            default_value="/fmu/out/vehicle_local_position"),

        Node(
            package="tarp_detection",
            executable="tarp_detection_node",
            name="tarp_detection",
            output="screen",
            parameters=[cfg, {
                "image_topic":       LaunchConfiguration("image_topic"),
                "global_pos_topic":  LaunchConfiguration("global_pos_topic"),
                "local_pos_topic":   LaunchConfiguration("local_pos_topic"),
            }],
        ),
    ])
