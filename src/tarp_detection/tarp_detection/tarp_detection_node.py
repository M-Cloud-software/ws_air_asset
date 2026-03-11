#!/usr/bin/env python3
"""
tarp_detection_node.py
ROS2 Humble — Jetson Orin Nano Super (JetPack 6.1)

Subscribes:
  /camera_feed                      (sensor_msgs/Image)
  /fmu/out/vehicle_global_position  (px4_msgs/VehicleGlobalPosition)
  /fmu/out/vehicle_local_position   (px4_msgs/VehicleLocalPosition)

Publishes:
  /detection_image     (sensor_msgs/Image)  — annotated frame
  /objects_of_interest (std_msgs/String)    — JSON string

Detection pipeline (per frame):
  1. Convert BGR -> HSV
  2. Threshold pixels within `color_tolerance` of target HSV -> binary mask
  3. connectedComponentsWithStats on mask -> blobs
  4. Filter blobs by [min_pixels, max_pixels]
  5. Project each blob centroid to GPS using drone altitude + camera FOV

JSON output shape:
  {
    "drone_lat": float,
    "drone_lon": float,
    "drone_alt": float,
    "detections": [
      {
        "label": "tarp",
        "bbox": [x1, y1, x2, y2],
        "pixel_center": [cx, cy],
        "pixel_count": int,
        "target_lat": float,
        "target_lon": float
      },
      ...
    ]
  }
"""

import json
import math
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from px4_msgs.msg import VehicleGlobalPosition, VehicleLocalPosition


class TarpDetectionNode(Node):

    def __init__(self):
        super().__init__('tarp_detection')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('image_topic',        '/camera_feed')
        self.declare_parameter('global_pos_topic',   '/fmu/out/vehicle_global_position')
        self.declare_parameter('local_pos_topic',    '/fmu/out/vehicle_local_position')
        self.declare_parameter('target_color_bgr',   [0, 0, 200])
        self.declare_parameter('color_tolerance',    35)
        self.declare_parameter('min_pixels',         500)
        self.declare_parameter('max_pixels',         200000)
        self.declare_parameter('camera_hfov_deg',    84.0)
        self.declare_parameter('camera_vfov_deg',    54.0)

        image_topic  = self.get_parameter('image_topic').value
        global_topic = self.get_parameter('global_pos_topic').value
        local_topic  = self.get_parameter('local_pos_topic').value
        color_bgr    = self.get_parameter('target_color_bgr').value
        self.tolerance   = self.get_parameter('color_tolerance').value
        self.min_pixels  = self.get_parameter('min_pixels').value
        self.max_pixels  = self.get_parameter('max_pixels').value
        self.hfov = math.radians(self.get_parameter('camera_hfov_deg').value)
        self.vfov = math.radians(self.get_parameter('camera_vfov_deg').value)

        # Convert target colour to HSV once at startup
        bgr_pixel       = np.uint8([[color_bgr]])
        self.target_hsv = cv2.cvtColor(bgr_pixel, cv2.COLOR_BGR2HSV)[0][0].astype(np.float32)

        self.bridge = CvBridge()

        # ── PX4 state ────────────────────────────────────────────────────────
        self.global_pos = None   # VehicleGlobalPosition
        self.local_pos  = None   # VehicleLocalPosition

        # ── QoS (matches PX4 uXRCE-DDS publisher) ────────────────────────────
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(Image,                 image_topic,  self._image_cb,      qos)
        self.create_subscription(VehicleGlobalPosition, global_topic, self._global_pos_cb, qos)
        self.create_subscription(VehicleLocalPosition,  local_topic,  self._local_pos_cb,  qos)

        # ── Publishers ────────────────────────────────────────────────────────
        self.img_pub = self.create_publisher(Image,  '/detection_image',     qos)
        self.det_pub = self.create_publisher(String, '/objects_of_interest', 10)

        self.get_logger().info('tarp_detection node ready')

    # ── PX4 callbacks ─────────────────────────────────────────────────────────
    def _global_pos_cb(self, msg):
        self.global_pos = msg

    def _local_pos_cb(self, msg):
        self.local_pos = msg

    # ── Main image callback ───────────────────────────────────────────────────
    def _image_cb(self, msg):
        image_ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        h, w = frame.shape[:2]

        # ── Step 1 & 2: HSV threshold -> binary mask ──────────────────────────
        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV).astype(np.float32)

        h_diff = np.abs(hsv[:, :, 0] - self.target_hsv[0])
        h_diff = np.minimum(h_diff, 180.0 - h_diff)   # hue wraps at 180
        s_diff = np.abs(hsv[:, :, 1] - self.target_hsv[1])
        v_diff = np.abs(hsv[:, :, 2] - self.target_hsv[2])
        dist   = np.sqrt((h_diff * 2.0) ** 2 + s_diff ** 2 + v_diff ** 2)

        mask = (dist <= self.tolerance).astype(np.uint8)

        # ── Step 3: Connected Component Analysis ──────────────────────────────
        num_labels, _, stats, centroids = cv2.connectedComponentsWithStats(
            mask, connectivity=8
        )

        # ── Step 4: Filter blobs + project GPS ───────────────────────────────
        detection_ts = time.time()
        propagated   = self._propagate_gps(image_ts, detection_ts)

        detections = []
        annotated  = frame.copy()

        for i in range(1, num_labels):   # 0 is background
            px_count = int(stats[i, cv2.CC_STAT_AREA])
            if not (self.min_pixels <= px_count <= self.max_pixels):
                continue

            x1 = int(stats[i, cv2.CC_STAT_LEFT])
            y1 = int(stats[i, cv2.CC_STAT_TOP])
            bw = int(stats[i, cv2.CC_STAT_WIDTH])
            bh = int(stats[i, cv2.CC_STAT_HEIGHT])
            x2 = x1 + bw
            y2 = y1 + bh
            cx = int(centroids[i][0])
            cy = int(centroids[i][1])

            det = {
                'label':        'tarp',
                'bbox':         [x1, y1, x2, y2],
                'pixel_center': [cx, cy],
                'pixel_count':  px_count,
                'target_lat':   0.0,
                'target_lon':   0.0,
            }

            if propagated:
                lat, lon, alt = propagated
                det['target_lat'], det['target_lon'] = _pixel_to_gps(
                    cx, cy, w, h, lat, lon, alt, self.hfov, self.vfov
                )

            detections.append(det)

            # Draw bounding box and centroid
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 230, 120), 2)
            cv2.circle(annotated, (cx, cy), 5, (0, 0, 255), -1)
            cv2.putText(annotated,
                        f'tarp {px_count}px',
                        (x1, max(y1 - 6, 0)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 230, 120), 2)
            if propagated:
                cv2.putText(annotated,
                            f'{det["target_lat"]:.5f}, {det["target_lon"]:.5f}',
                            (x1, y2 + 14),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 220, 0), 1)

        # ── Publish JSON ───────────────────────────────────────────────────────
        payload = {
            'drone_lat':  propagated[0] if propagated else 0.0,
            'drone_lon':  propagated[1] if propagated else 0.0,
            'drone_alt':  propagated[2] if propagated else 0.0,
            'detections': detections,
        }
        self.det_pub.publish(String(data=json.dumps(payload)))

        # ── Publish annotated image ────────────────────────────────────────────
        try:
            out = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            out.header = msg.header
            self.img_pub.publish(out)
        except Exception as e:
            self.get_logger().error(f'image publish error: {e}')

        self.get_logger().debug(f'{len(detections)} detection(s)')

    # ── GPS dead-reckoning ────────────────────────────────────────────────────
    def _propagate_gps(self, image_ts, detection_ts):
        """
        Propagate drone position from last known fix to detection time.
        Uses VehicleLocalPosition vx/vy (NED, m/s) from the PX4 EKF.
        Returns (lat, lon, alt) or None if no fix yet.
        """
        if self.global_pos is None:
            return None

        gps_ts = self.global_pos.timestamp * 1e-6   # microseconds -> seconds
        dt = (image_ts - gps_ts) + (detection_ts - image_ts)
        dt = max(dt, 0.0)

        vx = self.local_pos.vx if self.local_pos else 0.0   # North m/s
        vy = self.local_pos.vy if self.local_pos else 0.0   # East  m/s

        lat = self.global_pos.lat
        lon = self.global_pos.lon
        alt = self.global_pos.alt

        mlat = 111_320.0
        mlon = 111_320.0 * math.cos(math.radians(lat))

        return (lat + (vx / mlat) * dt,
                lon + (vy / mlon) * dt,
                alt)


# ── GPS projection (nadir camera) ─────────────────────────────────────────────
def _pixel_to_gps(px, py, w, h, drone_lat, drone_lon, alt, hfov, vfov):
    alt  = max(alt, 1.0)
    gsd_x = 2.0 * alt * math.tan(hfov / 2.0) / w
    gsd_y = 2.0 * alt * math.tan(vfov / 2.0) / h
    dx = (px - w / 2.0) * gsd_x   # metres East
    dy = (py - h / 2.0) * gsd_y   # metres South
    mlat = 111_320.0
    mlon = 111_320.0 * math.cos(math.radians(drone_lat))
    return (drone_lat - dy / mlat,
            drone_lon + dx / mlon)


def main(args=None):
    rclpy.init(args=args)
    node = TarpDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
