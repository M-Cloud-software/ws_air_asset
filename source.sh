#!/bin/bash
# ======================================
# PX4 + ROS2 + Gazebo Build Script
# Works on: WSL + Linux (VM/native)
# ======================================

if [ -d "/workspaces/ws_air_asset" ]; then
    WORKSPACE=/workspaces/ws_air_asset
else
    WORKSPACE=~/ws_air_asset
fi

cd "$WORKSPACE"
source /opt/ros/humble/setup.bash
# source install/local_setup.bash

# Only source the workspace if it's been built
if [ -f "install/local_setup.bash" ]; then
    source install/local_setup.bash
else
    echo "Workspace not yet built. Run 'colcon build' first."
fi