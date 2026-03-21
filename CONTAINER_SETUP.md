$ Container Setup for developing ws_air_asset

When you clone the ws_air_asset github, there are a variety of steps that need to be followed in order to start working on the project.

1. Download docker desktop on your device. 
* Mac Download [here](https://docs.docker.com/desktop/setup/install/mac-install/)
* Windows Download [here](https://docs.docker.com/desktop/setup/install/windows-install/)

2. On vscode: Download the "Dev Containers" extension (by Microsoft)

3. When prompted, click Reopen in Container — or manually: Cmd+Shift+P (mac) or Ctrl+Shift+P (windows) → Dev Containers: Rebuild and Reopen in Container

    * Wait for the image to build (first time only, takes several minutes). Once the terminal prompt appears, the environment is ready.

## After the Container Builds

### 1. Clone PX4 Autopilot (one-time)
```bash
cd /workspaces
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```

### 2. Build the workspace
```bash
cd /workspaces/ws_air_asset
./launch_project.sh build
```
This also auto-clones `px4_msgs` and `px4_ros_com` into `src/` if they are missing.

### 3. Source the workspace
Run this in every new terminal before using `ros2` commands:
```bash
source source.sh
```

### 4. Verify the build
```bash
ros2 pkg list | grep tarp_detection
ros2 pkg list | grep jetson_modem
```
Both packages should appear. If they don't, re-run step 2.

---

That's it — the container is ready. See the main README for how to run the simulation or deploy to hardware.
