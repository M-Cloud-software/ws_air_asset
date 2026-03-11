# MCLOUD Air Asset – ws_air_asset

Personal workspace for MCLOUD drone perception software. Built on ROS2 Humble, PX4, and Gazebo running on a Jetson Orin Nano Super (JetPack 6.1).

---

## Workspace Structure

```
ws_air_asset/
├── src/
│   ├── tarp_detection/       # Active — HSV + CCA object detection pipeline
│   ├── px4_msgs/             # Auto-cloned by launch_project.sh
│   └── px4_ros_com/          # Auto-cloned by launch_project.sh
├── launch_project.sh         # Main launch script (see below)
├── source.sh                 # Sources ROS2 + workspace (use before ros2 commands)
└── README.md
```

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

## Packages

### `tarp_detection` — Active development

Object detection pipeline for identifying tarps from a drone camera.

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
2. Compute per-pixel HSV distance from target colour
3. Threshold at `color_tolerance` → binary mask
4. `connectedComponentsWithStats` → blobs
5. Filter blobs by `[min_pixels, max_pixels]`
6. Project surviving blob centroids to GPS using drone altitude + camera FOV

**Subscribed topics:**

| Topic | Type | Source |
|---|---|---|
| `/camera_feed` | `sensor_msgs/Image` | Camera node (not yet built) |
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
| `target_color_bgr` | `[0, 0, 200]` | Target colour in BGR |
| `color_tolerance` | `35` | HSV distance threshold — raise to catch more, lower to reduce false positives |
| `min_pixels` | `500` | Minimum blob size |
| `max_pixels` | `200000` | Maximum blob size |
| `camera_hfov_deg` | `84.0` | Horizontal FOV of camera |
| `camera_vfov_deg` | `54.0` | Vertical FOV of camera |

**`sitl_publisher.py`** — test harness that runs without any hardware:
- Publishes a synthetic aerial frame with a coloured tarp blob embedded
- Publishes fake `VehicleGlobalPosition` and `VehicleLocalPosition` at correct rates
- Subscribes to `/objects_of_interest` and prints detections to terminal
- Accepts a real image via `image_path` parameter

---

### `px4_msgs` / `px4_ros_com` — PX4 bridge (auto-managed)

Cloned automatically by `launch_project.sh`. These provide the ROS2 message types and DDS bridge for communicating with PX4 SITL or hardware. Must be built before `tarp_detection` will run.

---

## Build & Run

```bash
# Build everything (required after first clone, or after code changes)
./launch_project.sh build

# Source the workspace in any new terminal
source source.sh

# Run detection pipeline against real hardware
ros2 launch tarp_detection detection.launch.py

# Run SITL test harness with randomly generated input (no drone required)
ros2 launch tarp_detection sitl.launch.py

# Run SITL test harness with supplied image (no drone required)
ros2 launch tarp_detection sitl.launch.py image_path:=/absolute/path/to/test_simple.png

# View annotated output
ros2 run rqt_image_view rqt_image_view /detection_image

# View GPS coordinates of the object in the image
ros2 topic echo /objects_of_interest
```

---

## Current Status

| Component | Status |
|---|---|
| `tarp_detection_node` | Working — builds and runs |
| `sitl_publisher` | Working — builds and runs |
| PX4 SITL + MicroXRCEAgent | Working via `launch_project.sh` |
| Camera node | Not yet built |
| Comms / modem node | Not yet built |

### Known Issues

Currently none.

---

## Pipeline Architecture (planned)

```
camera_node  ──→  /camera_feed  ──┐
                                   ├──→  tarp_detection_node  ──→  /objects_of_interest  ──→  jetson_modem
PX4 / MicroXRCEAgent  ────────────┘            │
  /fmu/out/vehicle_global_position              └──→  /detection_image
  /fmu/out/vehicle_local_position
```

Nodes not yet built: `camera_node`, `jetson_modem`.