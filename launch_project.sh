#!/bin/bash
# ======================================
# PX4 + ROS2 + Gazebo Launch Script
# ======================================

# Usage:
#   ./launch_project.sh          # Normal launch
#   ./launch_project.sh build    # Rebuild ws_air_asset
#   ./launch_project.sh quit     # Shut down all running PX4/Gazebo processes

set -Eeo pipefail

WORKSPACE=~/ws_air_asset
MODEL_PATH=$WORKSPACE/models

# -------- 0. Optional Build Step --------
if [[ "${1:-}" == "build" ]]; then
    echo "Building ws_air_asset workspace..."
    
    source /opt/ros/humble/setup.bash
    cd "$WORKSPACE"
    colcon build 
    
    echo "Build completed for ws_air_asset."
    exit 0
fi

echo "🚀 Starting PX4 + ROS2 + Gazebo system..."
echo

# -------- 1. Source Environments & ROS2 Workspace --------
cd "$WORKSPACE"
source /opt/ros/humble/setup.bash
source install/local_setup.bash

# Export model path for PX4/Gazebo
export GZ_SIM_RESOURCE_PATH=$MODEL_PATH
export PX4_SIM_MODEL_PATH=$MODEL_PATH
echo "✅ Environments sourced and model path set: $MODEL_PATH"
echo

# Give pc time for code to run (1 second)
sleep 1

# -------- 2. Launch PX4 SITL in New Terminal --------
echo "🛩️  Launching PX4 SITL..."
gnome-terminal -- bash -c "
  cd ~/PX4-Autopilot;
  source /opt/ros/humble/setup.bash;
  export GZ_SIM_RESOURCE_PATH=$MODEL_PATH;
  make px4_sitl gz_x500;
  exec bash
" &

# Give pc time for code to run (5 seconds)
sleep 5

# -------- 3. Launch MicroXRCEAgent in new terminal --------
echo "🔗 Starting MicroXRCEAgent..."
gnome-terminal -- bash -c "
  source /opt/ros/humble/setup.bash;
  MicroXRCEAgent udp4 -p 8888;
  exec bash
" &

