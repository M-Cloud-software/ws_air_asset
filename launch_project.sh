#!/bin/bash
# ======================================
# PX4 + ROS2 + Gazebo Launch Script
# Works on: WSL + Linux (VM/native)
# ======================================

set -Eeo pipefail

WORKSPACE=~/ws_air_asset
MODEL_PATH=$WORKSPACE/models

# -------- Detect px4_msgs and px4_ros_com --------
if [ ! -d "$WORKSPACE/src/px4_ros_com" ]; then
    echo "Cloning px4_ros_com..."
    git clone https://github.com/PX4/px4_ros_com.git "$WORKSPACE/src/px4_ros_com"
fi
if [ ! -d "$WORKSPACE/src/px4_msgs" ]; then
    echo "Cloning px4_msgs..."
    git clone https://github.com/PX4/px4_msgs.git "$WORKSPACE/src/px4_msgs"
fi

# -------- Detect WSL --------
if grep -qi "microsoft" /proc/version; then
    IS_WSL=true
else
    IS_WSL=false
fi

# -------- Terminal Launcher (Strict WSL-safe version) --------
launch_terminal() {
    local title="$1"
    local cmd="$2"

    if [ "$IS_WSL" = true ]; then
        wt.exe new-tab --title "$title" -- \
            wsl.exe -d "Ubuntu-22.04" --cd "$HOME" -- bash -lc "$cmd"
    else
        gnome-terminal -- bash -c "$cmd; exec bash"
    fi
}

# -------- 0. Build Step --------
    echo "Building workspace..."

    source /opt/ros/humble/setup.bash
    cd "$WORKSPACE"
    colcon build
    
    echo "Workspace installed and built! Exiting..."
    exit 0
# if [[ "$1" == "build" ]]; then
#     echo "Building workspace..."

#     source /opt/ros/humble/setup.bash
#     cd "$WORKSPACE"
#     colcon build
    
#     echo "Workspace installed and built! Exiting..."
#     exit 0
# fi


if [[ "$1" == "run" ]]; then
    echo "Starting system..."

    # -------- 1. Environment --------
    cd "$WORKSPACE"
    source /opt/ros/humble/setup.bash
    source install/local_setup.bash

    export GZ_SIM_RESOURCE_PATH="$MODEL_PATH"
    export PX4_SIM_MODEL_PATH="$MODEL_PATH"

    sleep 1

    # -------- 2. Launch PX4 SITL --------
    launch_terminal "PX4 SITL" \
    "cd ~/PX4-Autopilot && source /opt/ros/humble/setup.bash && export GZ_SIM_RESOURCE_PATH=$MODEL_PATH && make px4_sitl gz_x500"

    sleep 3

    # -------- 3. Launch MicroXRCEAgent --------
    launch_terminal "MicroXRCEAgent" \
    "source /opt/ros/humble/setup.bash && MicroXRCEAgent udp4 -p 8888"
