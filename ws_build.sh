#!/bin/bash
# ======================================
# PX4 + ROS2 + Gazebo Build Script
# Works on: WSL + Linux (VM/native)
# ======================================

WORKSPACE=~/ws_air_asset

cd "$WORKSPACE"
source /opt/ros/humble/setup.bash
source install/local_setup.bash