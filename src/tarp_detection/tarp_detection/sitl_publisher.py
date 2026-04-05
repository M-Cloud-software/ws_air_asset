#!/usr/bin/env python3
"""
sitl_publisher.py  —  SITL test harness

Publishes:
  /camera_feed                      (sensor_msgs/Image)          @ fps Hz
  /fmu/out/vehicle_global_position  (px4_msgs/VehicleGlobalPosition) @ 5 Hz
  /fmu/out/vehicle_local_position   (px4_msgs/VehicleLocalPosition)  @ 50 Hz

Subscribes (to print results):
  /objects_of_interest  (std_msgs/String)

Usage:
  ros2 launch tarp_detection sitl.launch.py
  ros2 launch tarp_detection sitl.launch.py image_path:=/path/to/image.jpg
  ros2 launch tarp_detection sitl.launch.py altitude_m:=40.0 speed_mps:=5.0
"""

import math
import os
import time

import cv2
import numpy as np
import yaml
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from px4_msgs.msg import VehicleGlobalPosition, VehicleLocalPosition


def _load_detection_params() -> dict:
    """
    Read detection_params.yaml directly from the installed package share
    directory and return the ros__parameters dict.

    This is the same file that tarp_detection_node reads at launch, so
    whatever colour is configured there is automatically what the SITL
    synthetic frame uses — no manual syncing required.
    """
    pkg = get_package_share_directory('tarp_detection')
    path = os.path.join(pkg, 'config', 'detection_params.yaml')
    with open(path, 'r') as f:
        cfg = yaml.safe_load(f)
    return cfg['tarp_detection']['ros__parameters']


def _make_synthetic_frame(width=640, height=480, tarp_color_bgr=(0, 0, 200)):
    """Noisy ground texture with one solid-colour tarp rectangle in the centre."""
    rng = np.random.default_rng(42)
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    frame[:, :, 0] = rng.integers(30,  80, (height, width), dtype=np.uint8)
    frame[:, :, 1] = rng.integers(70, 130, (height, width), dtype=np.uint8)
    frame[:, :, 2] = rng.integers(40,  90, (height, width), dtype=np.uint8)

    # Tarp blob — centred, ~10 % of frame area
    tx, ty = width // 3,  height // 3
    tw, th = width // 5,  height // 5
    noise = rng.integers(-15, 15, (th, tw, 3), dtype=np.int16)
    patch = (np.array(tarp_color_bgr, dtype=np.int16) + noise).clip(0, 255).astype(np.uint8)
    frame[ty:ty + th, tx:tx + tw] = patch
    return frame


class SITLPublisher(Node):

    def __init__(self):
        super().__init__('sitl_publisher')

        # ── Read detection_params.yaml directly ───────────────────────────────
        # We parse the file ourselves rather than going through the ROS2
        # parameter server, because detection_params.yaml is only loaded into
        # tarp_detection_node by the launch file — not into this node.
        # Reading the file directly means the SITL frame colour always matches
        # whatever tarp_detection_node is actually looking for.
        detection_params = _load_detection_params()
        tarp_color = tuple(detection_params['target_color_bgr'])

        self.get_logger().info(
            f'Read target_color_bgr={list(tarp_color)} from detection_params.yaml'
        )

        # ── ROS2 parameters (SITL-specific, not in detection_params.yaml) ─────
        self.declare_parameter('image_path',  '')
        self.declare_parameter('fps',         10.0)
        self.declare_parameter('start_lat',   37.7749)
        self.declare_parameter('start_lon',  -122.4194)
        self.declare_parameter('altitude_m',  30.0)
        self.declare_parameter('speed_mps',   3.0)
        self.declare_parameter('heading_deg', 90.0)

        image_path = self.get_parameter('image_path').value
        fps        = self.get_parameter('fps').value
        self.lat   = self.get_parameter('start_lat').value
        self.lon   = self.get_parameter('start_lon').value
        self.alt   = self.get_parameter('altitude_m').value
        speed      = self.get_parameter('speed_mps').value
        heading    = math.radians(self.get_parameter('heading_deg').value)

        # Velocity in deg/s along heading
        self._v_lat  = (speed * math.cos(heading)) / 111_320.0
        self._v_lon  = (speed * math.sin(heading)) / (111_320.0 * math.cos(math.radians(self.lat)))
        self._last_t = time.time()

        # ── Frame ─────────────────────────────────────────────────────────────
        if image_path:
            img = cv2.imread(image_path)
            if img is None:
                self.get_logger().warn(
                    f'Could not load {image_path} — falling back to synthetic frame'
                )
                img = _make_synthetic_frame(tarp_color_bgr=tarp_color)
        else:
            img = _make_synthetic_frame(tarp_color_bgr=tarp_color)
            self.get_logger().info('Using synthetic frame with embedded tarp blob')

        self._frame = img
        self.bridge = CvBridge()

        # ── QoS ───────────────────────────────────────────────────────────────
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Publishers ────────────────────────────────────────────────────────
        self._img_pub    = self.create_publisher(Image,                 '/camera_feed',                     qos)
        self._global_pub = self.create_publisher(VehicleGlobalPosition, '/fmu/out/vehicle_global_position', qos)
        self._local_pub  = self.create_publisher(VehicleLocalPosition,  '/fmu/out/vehicle_local_position',  qos)

        # ── Subscriber — print detections ─────────────────────────────────────
        self.create_subscription(String, '/objects_of_interest', self._det_cb, 10)

        # ── Timers ────────────────────────────────────────────────────────────
        self.create_timer(1.0 / fps,   self._pub_image)
        self.create_timer(1.0 / 5.0,   self._pub_global)
        self.create_timer(1.0 / 50.0,  self._pub_local)

        self.get_logger().info(
            f'SITL ready — fps={fps} start=({self.lat:.5f}, {self.lon:.5f}) alt={self.alt}m'
        )

    def _pub_image(self):
        msg = self.bridge.cv2_to_imgmsg(self._frame, encoding='bgr8')
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        self._img_pub.publish(msg)

    def _pub_global(self):
        now = time.time()
        dt  = now - self._last_t
        self.lat    += self._v_lat * dt
        self.lon    += self._v_lon * dt
        self._last_t = now

        msg = VehicleGlobalPosition()
        msg.timestamp     = int(now * 1e6)
        msg.lat           = self.lat
        msg.lon           = self.lon
        msg.alt           = self.alt
        msg.alt_ellipsoid = self.alt
        msg.eph           = 0.5
        msg.epv           = 0.8
        self._global_pub.publish(msg)

    def _pub_local(self):
        msg = VehicleLocalPosition()
        msg.timestamp  = int(time.time() * 1e6)
        msg.vx         = self._v_lat * 111_320.0
        msg.vy         = self._v_lon * 111_320.0 * math.cos(math.radians(self.lat))
        msg.vz         = 0.0
        msg.z          = -self.alt
        msg.xy_valid   = True
        msg.z_valid    = True
        msg.v_xy_valid = True
        msg.v_z_valid  = True
        self._local_pub.publish(msg)

    def _det_cb(self, msg):
        import json
        data = json.loads(msg.data)
        n = len(data['detections'])
        if n == 0:
            return
        self.get_logger().info(
            f'[SITL] {n} detection(s) | '
            f'drone=({data["drone_lat"]:.6f}, {data["drone_lon"]:.6f}, {data["drone_alt"]:.1f}m)'
        )
        for i, d in enumerate(data['detections']):
            self.get_logger().info(
                f'  [{i}] {d["label"]}  pixels={d["pixel_count"]}  '
                f'center={d["pixel_center"]}  '
                f'target=({d["target_lat"]:.6f}, {d["target_lon"]:.6f})'
            )


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


if __name__ == '__main__':
    main()
