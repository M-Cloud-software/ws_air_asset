"""
sitl.launch.py  –  starts the SITL publisher AND detector together (PX4 uXRCE-DDS)

ros2 launch tarp_detection sitl.launch.py
ros2 launch tarp_detection sitl.launch.py \
    image_path:=/path/to/test.jpg \
    start_lat:=37.7749 start_lon:=-122.4194 altitude_m:=25.0
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
        # ── SITL tuning args ─────────────────────────────────────────────
        DeclareLaunchArgument("image_path",  default_value=""),
        DeclareLaunchArgument("start_lat",   default_value="37.7749"),
        DeclareLaunchArgument("start_lon",   default_value="-122.4194"),
        DeclareLaunchArgument("altitude_m",  default_value="30.0"),
        DeclareLaunchArgument("speed_mps",   default_value="3.0"),
        DeclareLaunchArgument("fps",         default_value="10.0"),

        # ── SITL hardware simulator ───────────────────────────────────────
        Node(
            package="tarp_detection",
            executable="sitl_publisher",
            name="sitl_publisher",
            output="screen",
            parameters=[{
                "image_path":  LaunchConfiguration("image_path"),
                "start_lat":   LaunchConfiguration("start_lat"),
                "start_lon":   LaunchConfiguration("start_lon"),
                "altitude_m":  LaunchConfiguration("altitude_m"),
                "speed_mps":   LaunchConfiguration("speed_mps"),
                "fps":         LaunchConfiguration("fps"),
            }],
        ),

        # ── Detection pipeline ────────────────────────────────────────────
        Node(
            package="tarp_detection",
            executable="tarp_detection_node",
            name="tarp_detection",
            output="screen",
            parameters=[cfg],
        ),
    ])
