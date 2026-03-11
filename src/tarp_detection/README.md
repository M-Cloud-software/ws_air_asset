# tarp_detection

Pure-OpenCV tarp detection pipeline for **ROS2 Humble** on the **Jetson Orin Nano Super (JetPack 6.1)**.  
No YOLO, no neural network — gradient-enhanced colour blob detection only.

---

## Architecture

```
/camera_feed  (sensor_msgs/Image)    ──┐
                                        ├──▶  tarp_detection  ──▶  /objects_of_interest
/drone_pos    (sensor_msgs/NavSatFix) ──┘                     ──▶  /detection_image
```

The **SITL publisher** replaces both hardware nodes for testing:

```
sitl_publisher  ──▶  /camera_feed   ──┐
                ──▶  /drone_pos      ──┴──▶  tarp_detection  ──▶  /objects_of_interest
                ◀──  /objects_of_interest        (printed to terminal)
```

---

## Detection Algorithm

### Stage 1 — Colour mask
Every pixel's distance from the target colour is computed in HSV space (default) or BGR:

```
HSV distance = sqrt( (2·ΔH)² + ΔS² + ΔV² )
```
Pixels within `color_tolerance` are set to 1 in the colour mask.

### Stage 2 — Gradient (Sobel) refinement
Sobel X/Y are computed on the greyscale frame.  
Edge magnitude = sqrt(Sx² + Sy²), normalised to 0-255.  
Edges exceeding `gradient_threshold` that overlap a *dilated* colour mask are
included — this captures sharp object boundaries just outside the colour blob,
giving tighter, less noisy bounding boxes.

### Stage 3 — Morphological close + CCA
```
combined = color_mask OR (gradient_weight × grad_near_color_mask)
combined = morphClose(combined, ellipse_kernel_7x7)
```
Connected Component Analysis (8-connectivity) extracts blobs; those outside
`[min_pixels, max_pixels]` are discarded.

### GPS projection (nadir camera, dead-reckoned)

```
GSD_x = 2 · alt · tan(HFOV/2) / image_width       [m/pixel]
GSD_y = 2 · alt · tan(VFOV/2) / image_height

dx = (pixel_cx - W/2) · GSD_x    [metres East  of drone]
dy = (pixel_cy - H/2) · GSD_y    [metres South of drone]

target_lat = drone_lat_propagated - dy / 111_320
target_lon = drone_lon_propagated + dx / (111_320 · cos(lat))
```

Dead-reckoning propagates the drone position forward by:
`Δt = (image_ROS_stamp → GPS age) + (image_stamp → detection wall-clock)`

---

## Installation

```bash
# ROS2 Humble (ships with JetPack 6.1)
source /opt/ros/humble/setup.bash

# cv_bridge
sudo apt install -y ros-humble-cv-bridge ros-humble-vision-opencv

# Build
mkdir -p ~/ros2_ws/src
cp -r tarp_detection ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --packages-select tarp_detection
source install/setup.bash
```

---

## Running on real hardware

```bash
ros2 launch tarp_detection detection.launch.py

# Remap for MAVROS / your camera driver:
ros2 launch tarp_detection detection.launch.py \
  image_topic:=/camera/image_raw \
  gps_topic:=/mavros/global_position/raw/fix
```

---

## SITL — Testing Without Hardware

The `sitl_publisher` node synthesises camera frames and GPS fixes so you can
validate the entire pipeline on any Ubuntu machine (or the Jetson itself, before
plugging in camera/GPS).

### What SITL simulates
| Real hardware | SITL equivalent |
|---|---|
| `camera_node` → `/camera_feed` | Publishes a generated frame with an embedded tarp blob at configurable FPS |
| `gps_node` → `/drone_pos` | Publishes NavSatFix at 5 Hz; drone position moves along a heading at a set speed |
| Observation | Subscribes to `/objects_of_interest` and prints every detection to stdout |

### Quickstart (synthetic frame, default params)

**Terminal 1 — launch everything together:**
```bash
ros2 launch tarp_detection sitl.launch.py
```

You will immediately see log output like:
```
[sitl_publisher]: Using synthetic test frame with embedded tarp blob
[tarp_detection]:  TarpDetectionNode ready  (gradient + colour, no ML)
[sitl_publisher]: [SITL] 1 detection(s) | drone=(37.774900, -122.419400, 30.0m)
[sitl_publisher]:   [0] tarp  conf=1.00  px=(213,160)  gradE=28.3  target=(37.774856, -122.419287)
```

### Supplying your own test image
```bash
ros2 launch tarp_detection sitl.launch.py \
  image_path:=/home/jetson/test_images/field_photo.jpg
```

### Override drone position / flight profile
```bash
ros2 launch tarp_detection sitl.launch.py \
  start_lat:=47.6062  start_lon:=-122.3321 \
  altitude_m:=50.0    speed_mps:=8.0       fps:=15.0
```

### Run nodes separately (useful for debugging one at a time)

Terminal 1 — detection node only:
```bash
ros2 run tarp_detection tarp_detection_node
```

Terminal 2 — SITL publisher only:
```bash
ros2 run tarp_detection sitl_publisher \
  --ros-args -p altitude_m:=40.0 -p tarp_color_bgr:="[0,0,200]"
```

### Inspect topics manually
```bash
# See raw detections
ros2 topic echo /objects_of_interest

# Check message rate
ros2 topic hz /detection_image
ros2 topic hz /objects_of_interest

# View annotated image in RViz2 or rqt
ros2 run rqt_image_view rqt_image_view /detection_image
```

### Tune detector parameters live (no rebuild needed)
```bash
ros2 param set /tarp_detection color_tolerance 45
ros2 param set /tarp_detection gradient_threshold 30
ros2 param set /tarp_detection min_pixels 300
```

---

## Custom Messages

### `DetectionArray`
```
std_msgs/Header header
float64 drone_lat
float64 drone_lon
float64 drone_alt           # metres AGL
tarp_detection/Detection[] detections
```

### `Detection`
```
string  label               # always "tarp"
float32 confidence          # pixel_count / min_pixels, clamped 0–1
int32   bbox_x1, bbox_y1, bbox_x2, bbox_y2
int32   pixel_cx, pixel_cy
float32 gradient_energy     # mean Sobel magnitude inside bbox (blob quality)
float64 target_lat
float64 target_lon
```

---

## Tuning Guide

| Parameter | Effect | Start here if… |
|---|---|---|
| `color_tolerance` | ↑ catches more pixels, ↑ false positives | Missing detections in shade |
| `use_hsv` | HSV more robust to lighting changes than BGR | Lighting varies a lot |
| `gradient_threshold` | ↑ only very sharp edges, ↓ faint edges included | Boxes are too loose or noisy |
| `gradient_weight` | 0 = colour only; 1 = edges dominate | Bounding boxes too large |
| `morph_kernel_size` | Larger = fills bigger holes in mask | Fragmented blobs |
| `min_pixels` | Raise to ignore small clutter | Too many false positives |
| `camera_hfov_deg` | **Must match your lens** | GPS coords are offset from truth |

---

## MAVROS GPS topic remapping

```python
# in detection.launch.py remappings list:
('/drone_pos', '/mavros/global_position/raw/fix'),
```
