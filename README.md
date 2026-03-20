# MCLOUD Air Asset – ws_air_asset

Personal workspace for MCLOUD drone perception software. Built on ROS2 Humble, PX4, and Gazebo running on a Jetson Orin Nano Super (JetPack 6.1).

---

## Workspace Structure

```
ws_air_asset/
├── .devcontainer/
│   ├── devcontainer.json         # VS Code devcontainer config
│   └── Dockerfile                # Container image definition
├── src/
│   ├── tarp_detection/       # HSV + CCA object detection pipeline
│   ├── jetson_modem/         # Cellular transmitter — sends detections to ground station
│   ├── px4_msgs/             # Auto-cloned by launch_project.sh
│   └── px4_ros_com/          # Auto-cloned by launch_project.sh
├── launch_project.sh         # Main launch script (see below)
├── source.sh                 # Sources ROS2 + workspace (use before ros2 commands)
└── README.md
```

---

## Dev Container Setup

The workspace includes a VS Code devcontainer that provides the full development environment — ROS2 Humble, Micro XRCE-DDS Agent, and all vision dependencies — without any manual installation.

### Prerequisites

- Docker installed and running (`docker ps` to verify)
- VS Code with the **Dev Containers** extension installed

### First-time setup

1. Clone the repo and open the folder in VS Code:
   ```bash
   git clone git@github.com:M-Cloud-software/ws_air_asset.git
   cd ws_air_asset
   code .
   ```

2. When prompted, click **Reopen in Container** — or manually:
   `Ctrl+Shift+P` → **Dev Containers: Rebuild and Reopen in Container**

3. Wait for the image to build (first time only, takes several minutes). Once the terminal prompt appears, the environment is ready.

4. Clone PX4 Autopilot inside the container (one-time, not committed to the repo):
   ```bash
   cd /workspaces
   git clone https://github.com/PX4/PX4-Autopilot.git --recursive
   ```

### What the container includes

| Component | Details |
|---|---|
| ROS2 Humble | Auto-sourced in every terminal |
| Micro XRCE-DDS Agent | Pre-built, available as `MicroXRCEAgent` |
| cv-bridge, image-transport, vision-msgs | ROS2 vision packages |
| OpenCV | Available via `/opt/px4-venv` |
| colcon, ros-dev-tools | Build tools |

> ⚠️ **Jetson note:** Do not install PyTorch or torchvision via pip inside the container. Use NVIDIA's pre-built ARM64 wheels from the [Jetson PyTorch forum](https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048) if GPU-accelerated inference is needed.

### Returning to the container

The container persists between sessions. To reopen it:

`Ctrl+Shift+P` → **Dev Containers: Reopen in Container**

To rebuild from scratch (e.g. after Dockerfile changes):

`Ctrl+Shift+P` → **Dev Containers: Rebuild and Reopen in Container**

### What goes to GitHub / what doesn't

The **Dockerfile and devcontainer.json are committed** — anyone who clones the repo can rebuild the exact same environment. The built Docker image itself is not pushed anywhere; each machine builds it locally from the Dockerfile.

`px4_msgs`, `px4_ros_com`, and `PX4-Autopilot` are **not committed** — they are either auto-cloned by `launch_project.sh` or cloned manually as described above.

---

## Scripts

### `source.sh`
Sources ROS2 Humble and the workspace install. Run this in any terminal before using `ros2` commands.
```bash
source source.sh
```

### `launch_project.sh`
Auto-clones `px4_msgs` and `px4_ros_com` if missing, then starts the full simulation stack in separate terminals:
- **PX4 SITL** — `make px4_sitl gz_x500` in `~/PX4-Autopilot`
- **MicroXRCEAgent** — bridges PX4 uXRCE-DDS to ROS2 over UDP port 8888

```bash
./launch_project.sh           # start PX4 SITL + MicroXRCEAgent
./launch_project.sh build     # colcon build the entire workspace
```

Works on both native Linux and WSL (opens new tabs in Windows Terminal).

---

## Pipeline Architecture

```
camera_node  ──→  /camera_feed  ──┐
                                   ├──→  tarp_detection_node  ──→  /objects_of_interest  ──→  jetson_modem_node
PX4 / MicroXRCEAgent  ────────────┘            │                                                     │
  /fmu/out/vehicle_global_position              └──→  /detection_image               LTE (Sixfab modem)
  /fmu/out/vehicle_local_position                                                            │
                                                                             ground_station_server.py (laptop)
```

---

## Packages

### `tarp_detection`

Object detection pipeline. Subscribes to the camera feed and PX4 position topics, detects tarps using HSV color thresholding and connected component analysis, and publishes annotated images and GPS coordinates.

```
tarp_detection/
├── tarp_detection/
│   ├── tarp_detection_node.py    # Detection node
│   └── sitl_publisher.py         # Hardware-free test harness
├── launch/
│   ├── sitl.launch.py            # Runs both nodes for desktop testing
│   └── detection.launch.py       # Production — detection node only
├── config/
│   └── detection_params.yaml     # All tunable parameters
├── package.xml
├── setup.py
└── setup.cfg
```

**Detection pipeline** (per frame):
1. Convert BGR → HSV
2. Compute per-pixel HSV distance from target color
3. Threshold at `color_tolerance` → binary mask
4. `connectedComponentsWithStats` → blobs
5. Filter blobs by `[min_pixels, max_pixels]`
6. Project surviving blob centroids to GPS using drone altitude + camera FOV

**Subscribed topics:**

| Topic | Type | Source |
|---|---|---|
| `/camera_feed` | `sensor_msgs/Image` | Camera node |
| `/fmu/out/vehicle_global_position` | `px4_msgs/VehicleGlobalPosition` | PX4 via MicroXRCEAgent |
| `/fmu/out/vehicle_local_position` | `px4_msgs/VehicleLocalPosition` | PX4 via MicroXRCEAgent |

**Published topics:**

| Topic | Type | Contents |
|---|---|---|
| `/detection_image` | `sensor_msgs/Image` | Annotated frame with bounding boxes |
| `/objects_of_interest` | `std_msgs/String` | JSON — drone GPS + detection list |

**JSON output shape:**
```json
{
  "drone_lat": 37.7749,
  "drone_lon": -122.4194,
  "drone_alt": 30.0,
  "detections": [
    {
      "label": "tarp",
      "bbox": [x1, y1, x2, y2],
      "pixel_center": [cx, cy],
      "pixel_count": 1840,
      "target_lat": 37.7748,
      "target_lon": -122.4192
    }
  ]
}
```

**Key parameters** (`config/detection_params.yaml`):

| Parameter | Default | Notes |
|---|---|---|
| `target_color_bgr` | `[0, 0, 200]` | Target color in BGR |
| `color_tolerance` | `35` | HSV distance threshold — raise to catch more, lower to reduce false positives |
| `min_pixels` | `500` | Minimum blob size in pixels |
| `max_pixels` | `200000` | Maximum blob size in pixels |
| `camera_hfov_deg` | `84.0` | Horizontal FOV of camera lens |
| `camera_vfov_deg` | `54.0` | Vertical FOV of camera lens |

**`sitl_publisher.py`** — desktop test harness, no hardware required:
- Publishes a synthetic aerial frame with a colored tarp blob embedded, or a real image via `image_path`
- Publishes fake `VehicleGlobalPosition` and `VehicleLocalPosition` at correct PX4 rates
- Subscribes to `/objects_of_interest` and prints detections to terminal

---

### `jetson_modem`

Transmits detection results to the ground station laptop over the Sixfab LTE cellular modem. Only fires when a frame contains at least one detection — idle frames are ignored.

```
jetson_modem/
├── jetson_modem/
│   └── jetson_modem_node.py    # Modem transmitter node
├── launch/
│   └── modem.launch.py         # Runs tarp_detection + jetson_modem together
├── config/
│   └── modem_params.yaml       # Server IP, port, JPEG quality, timeout
├── package.xml
├── setup.py
└── setup.cfg
```

**Behaviour:**
- Subscribes to `/objects_of_interest` and `/detection_image`
- Waits until both arrive for the same frame
- If detections are non-empty, HTTP POSTs a multipart request to the ground station containing the JPEG image and JSON payload
- Send happens in a background thread — does not block the ROS2 callback loop

**Subscribed topics:**

| Topic | Type | Source |
|---|---|---|
| `/objects_of_interest` | `std_msgs/String` | `tarp_detection_node` |
| `/detection_image` | `sensor_msgs/Image` | `tarp_detection_node` |

**Key parameters** (`config/modem_params.yaml`):

| Parameter | Default | Notes |
|---|---|---|
| `server_ip` | `192.168.1.100` | Laptop IP — update before flying |
| `server_port` | `8080` | Must match the port `ground_station_server.py` is listening on |
| `jpeg_quality` | `80` | JPEG compression 0–100 |
| `timeout_sec` | `5.0` | HTTP request timeout |

---

### `px4_msgs` / `px4_ros_com` — PX4 bridge (auto-managed)

Cloned automatically by `launch_project.sh`. Provides ROS2 message types and the DDS bridge for communicating with PX4 SITL or hardware. Must be built before `tarp_detection` will run.

---

## Build & Run

```bash
# Build everything (required after first clone, or after any code changes)
./launch_project.sh build

# Source the workspace in any new terminal before running ros2 commands
source source.sh

# ── SITL testing (no hardware required) ──────────────────────────────────────

# Run with synthetic frame
ros2 launch tarp_detection sitl.launch.py

# Run with a real image
ros2 launch tarp_detection sitl.launch.py image_path:=/absolute/path/to/image.png

# View annotated output image
ros2 run rqt_image_view rqt_image_view    # then select /detection_image from dropdown

# View GPS coordinates in terminal
ros2 topic echo /objects_of_interest

# ── Real hardware ─────────────────────────────────────────────────────────────

# Detection only (no modem)
ros2 launch tarp_detection detection.launch.py

# Detection + modem transmission to ground station
ros2 launch jetson_modem modem.launch.py server_ip:=<laptop_ip>

# ── Ground station (run on laptop) ───────────────────────────────────────────
python3 ground_station_server.py --port 8080 --save-dir ./detections
```

---

## Current Status

| Component | Status |
|---|---|
| `tarp_detection_node` | Working |
| `sitl_publisher` | Working |
| `jetson_modem_node` | Built — untested (pending hardware) |
| `ground_station_server.py` | Built — untested (pending hardware) |
| PX4 SITL + MicroXRCEAgent | Working via `launch_project.sh` |
| Camera node (IMX477) | Built — untested (pending hardware) |