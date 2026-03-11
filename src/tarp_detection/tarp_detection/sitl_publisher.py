#!/usr/bin/env python3
"""
sitl_publisher.py  –  Software-In-The-Loop test harness  (PX4 uXRCE-DDS)

Simulates the hardware interface layer so you can test the full detection
pipeline on your development machine without a drone or PX4 hardware.

What it does:
  1. Generates or loads a test image (with a synthetic coloured tarp blob)
  2. Publishes  /camera_feed                        at a configurable FPS
  3. Publishes  /fmu/out/vehicle_global_position    at 5 Hz  (lat/lon/alt)
  4. Publishes  /fmu/out/vehicle_local_position     at 50 Hz (vx/vy/vz NED)
     — simulates a straight-line flight at constant speed and heading
  5. Subscribes to /objects_of_interest and prints detections to the terminal

Usage:
  # Synthetic tarp image (default):
  ros2 run tarp_detection sitl_publisher

  # Supply your own image:
  ros2 run tarp_detection sitl_publisher \
      --ros-args -p image_path:=/path/to/test.jpg

  # Override drone start position or speed:
  ros2 run tarp_detection sitl_publisher \
      --ros-args -p start_lat:=37.7749 -p start_lon:=-122.4194 \
                 -p speed_mps:=5.0 -p altitude_m:=30.0
"""

import math
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from px4_msgs.msg import VehicleGlobalPosition, VehicleLocalPosition

from tarp_detection.msg import DetectionArray


# ─────────────────────────────────────────────────────────────────────────────
def _make_synthetic_frame(width: int = 640, height: int = 480,
                           tarp_color_bgr: tuple = (0, 0, 200)) -> np.ndarray:
    """
    Create a fake aerial-looking frame with:
      - Noisy green/brown ground texture
      - One rectangular colour tarp blob near the centre
      - Random 'clutter' patches of similar-ish colours to stress the detector
    """
    rng = np.random.default_rng(42)

    # Ground texture: noisy greens / browns
    base = rng.integers(60, 120, (height, width, 3), dtype=np.uint8)
    base[:, :, 0] = rng.integers(30,  80, (height, width), dtype=np.uint8)  # B
    base[:, :, 1] = rng.integers(70, 130, (height, width), dtype=np.uint8)  # G
    base[:, :, 2] = rng.integers(40,  90, (height, width), dtype=np.uint8)  # R

    # Tarp: centred, ~10 % of frame area
    tx, ty = width // 3, height // 3
    tw, th = width // 5, height // 5
    tarp_noise = rng.integers(-15, 15, (th, tw, 3), dtype=np.int16)
    tarp_patch = (
        np.array(tarp_color_bgr, dtype=np.int16)[None, None, :] + tarp_noise
    ).clip(0, 255).astype(np.uint8)
    base[ty : ty + th, tx : tx + tw] = tarp_patch

    # A few clutter rectangles
    for _ in range(4):
        cx_ = rng.integers(0, width  - 40)
        cy_ = rng.integers(0, height - 40)
        cw_ = rng.integers(20, 60)
        ch_ = rng.integers(20, 50)
        clutter_color = rng.integers(0, 255, 3, dtype=np.uint8)
        base[cy_ : cy_ + ch_, cx_ : cx_ + cw_] = clutter_color

    return base


# ─────────────────────────────────────────────────────────────────────────────
class SITLPublisher(Node):

    def __init__(self):
        super().__init__("sitl_publisher")

        # ── Parameters ───────────────────────────────────────────────────
        self.declare_parameter("image_path",   "")           # "" → synthetic
        self.declare_parameter("fps",          10.0)
        self.declare_parameter("start_lat",    37.7749)
        self.declare_parameter("start_lon",    -122.4194)
        self.declare_parameter("altitude_m",   30.0)
        self.declare_parameter("speed_mps",    3.0)          # horizontal speed
        self.declare_parameter("heading_deg",  90.0)         # flight direction
        self.declare_parameter("tarp_color_bgr", [0, 0, 200])

        image_path   = self.get_parameter("image_path").value
        fps          = self.get_parameter("fps").value
        self.lat     = self.get_parameter("start_lat").value
        self.lon     = self.get_parameter("start_lon").value
        self.alt     = self.get_parameter("altitude_m").value
        speed        = self.get_parameter("speed_mps").value
        heading_deg  = self.get_parameter("heading_deg").value
        tarp_color   = self.get_parameter("tarp_color_bgr").value

        # Velocity in degrees/second
        heading_rad   = math.radians(heading_deg)
        self._v_lat   = (speed * math.cos(heading_rad)) / 111_320.0
        self._v_lon   = (speed * math.sin(heading_rad)) / (111_320.0 * math.cos(math.radians(self.lat)))
        self._last_gps_time = time.time()

        # ── Load / generate test image ────────────────────────────────────
        if image_path:
            img = cv2.imread(image_path)
            if img is None:
                self.get_logger().warn(f"Could not load {image_path}, using synthetic frame")
                img = _make_synthetic_frame(tarp_color_bgr=tuple(tarp_color))
        else:
            img = _make_synthetic_frame(tarp_color_bgr=tuple(tarp_color))
            self.get_logger().info("Using synthetic test frame with embedded tarp blob")

        self._frame = img
        self.bridge = CvBridge()

        # ── QoS ──────────────────────────────────────────────────────────
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Publishers ───────────────────────────────────────────────────
        self._img_pub    = self.create_publisher(Image,
            "/camera_feed", px4_qos)
        self._global_pub = self.create_publisher(VehicleGlobalPosition,
            "/fmu/out/vehicle_global_position", px4_qos)
        self._local_pub  = self.create_publisher(VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",  px4_qos)

        # ── Subscribers (observe detection output) ────────────────────────
        self.create_subscription(
            DetectionArray, "/objects_of_interest",
            self._det_cb, 10
        )
        self.create_subscription(
            Image, "/detection_image",
            self._annotated_cb, px4_qos
        )

        # ── Timers ───────────────────────────────────────────────────────
        self.create_timer(1.0 / fps,   self._pub_image)
        self.create_timer(1.0 / 5.0,   self._pub_global_pos)   # 5 Hz
        self.create_timer(1.0 / 50.0,  self._pub_local_pos)    # 50 Hz matches EKF output rate

        self.get_logger().info(
            f"SITL publisher ready  |  fps={fps}  "
            f"start=({self.lat:.5f}, {self.lon:.5f})  alt={self.alt}m"
        )

    # ── Publish camera frame ──────────────────────────────────────────────
    def _pub_image(self):
        try:
            ros_img          = self.bridge.cv2_to_imgmsg(self._frame, encoding="bgr8")
            ros_img.header.stamp    = self.get_clock().now().to_msg()
            ros_img.header.frame_id = "camera"
            self._img_pub.publish(ros_img)
        except Exception as exc:
            self.get_logger().error(f"image publish: {exc}")

    # ── Publish VehicleGlobalPosition (EKF fused, 5 Hz) ──────────────────
    def _pub_global_pos(self):
        now      = time.time()
        dt       = now - self._last_gps_time
        self.lat += self._v_lat * dt
        self.lon += self._v_lon * dt
        self._last_gps_time = now

        msg = VehicleGlobalPosition()
        # PX4 uses microseconds since boot for timestamp
        msg.timestamp = int(now * 1e6)
        msg.lat       = self.lat
        msg.lon       = self.lon
        msg.alt       = self.alt        # AMSL metres
        msg.alt_ellipsoid = self.alt
        msg.eph       = 0.5             # horizontal position uncertainty [m]
        msg.epv       = 0.8             # vertical position uncertainty   [m]
        self._global_pub.publish(msg)

    # ── Publish VehicleLocalPosition (EKF NED velocity, 50 Hz) ───────────
    def _pub_local_pos(self):
        msg = VehicleLocalPosition()
        msg.timestamp = int(time.time() * 1e6)
        # NED velocity: vx=North, vy=East, vz=Down
        # Convert deg/s back to m/s for the local position message
        msg.vx  = self._v_lat * 111_320.0                                          # North m/s
        msg.vy  = self._v_lon * 111_320.0 * math.cos(math.radians(self.lat))       # East  m/s
        msg.vz  = 0.0                                                               # Down  m/s (level flight)
        msg.z   = -self.alt     # NED: altitude is negative Z
        msg.xy_valid = True
        msg.z_valid  = True
        msg.v_xy_valid = True
        msg.v_z_valid  = True
        self._local_pub.publish(msg)

    # ── Detection output callbacks (SITL monitoring) ──────────────────────
    def _det_cb(self, msg: DetectionArray):
        n = len(msg.detections)
        if n == 0:
            return
        self.get_logger().info(
            f"[SITL] {n} detection(s) | "
            f"drone=({msg.drone_lat:.6f}, {msg.drone_lon:.6f}, {msg.drone_alt:.1f}m)"
        )
        for i, d in enumerate(msg.detections):
            self.get_logger().info(
                f"  [{i}] {d.label}  conf={d.confidence:.2f}  "
                f"px=({d.pixel_cx},{d.pixel_cy})  "
                f"gradE={d.gradient_energy:.1f}  "
                f"target=({d.target_lat:.6f}, {d.target_lon:.6f})"
            )

    def _annotated_cb(self, msg: Image):
        # Just log that annotated frames are arriving
        self.get_logger().debug("Annotated frame received")


# ─────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = SITLPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
