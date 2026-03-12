#!/usr/bin/env python3
"""
jetson_modem_node.py
ROS2 Humble — Jetson Orin Nano Super (JetPack 6.1)

Subscribes:
  /objects_of_interest  (std_msgs/String)   — JSON from tarp_detection_node
  /detection_image      (sensor_msgs/Image) — annotated frame

Behaviour:
  - Waits until both topics have delivered a message for the same frame
  - If detections list is non-empty, HTTP POSTs to the ground station
  - Does nothing if the frame had zero detections

POST endpoint:  http://<server_ip>:<server_port>/detection
POST body:      multipart/form-data
  - field "json"  : the raw JSON string from /objects_of_interest
  - field "image" : JPEG-encoded annotated image

Parameters (modem_params.yaml):
  server_ip    — IP address of the ground station laptop
  server_port  — port the ground station server is listening on (default 8080)
  jpeg_quality — JPEG compression quality 0-100 (default 80)
  timeout_sec  — HTTP request timeout in seconds (default 5.0)
"""

import json
import io
import threading
import urllib.request
import urllib.error

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String


class JetsonModemNode(Node):

    def __init__(self):
        super().__init__('jetson_modem')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('server_ip',    '192.168.1.100')
        self.declare_parameter('server_port',  8080)
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('timeout_sec',  5.0)

        self._server_ip   = self.get_parameter('server_ip').value
        self._server_port = self.get_parameter('server_port').value
        self._jpeg_quality = self.get_parameter('jpeg_quality').value
        self._timeout     = self.get_parameter('timeout_sec').value

        self._url = f'http://{self._server_ip}:{self._server_port}/detection'
        self.get_logger().info(f'Ground station URL: {self._url}')

        self._bridge = CvBridge()

        # ── Frame sync state ──────────────────────────────────────────────────
        # Both callbacks store their latest message; the second to arrive
        # triggers the send attempt for that pair.
        self._lock       = threading.Lock()
        self._latest_json  = None   # raw JSON string
        self._latest_image = None   # cv2 BGR frame
        self._json_stamp   = None   # (sec, nanosec) of latest JSON msg
        self._image_stamp  = None   # (sec, nanosec) of latest image msg

        # ── QoS ───────────────────────────────────────────────────────────────
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(String, '/objects_of_interest', self._json_cb,  10)
        self.create_subscription(Image,  '/detection_image',     self._image_cb, qos)

        self.get_logger().info('jetson_modem node ready — waiting for detections')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _json_cb(self, msg):
        with self._lock:
            self._latest_json  = msg.data
            self._json_stamp   = (msg.header.stamp.sec, msg.header.stamp.nanosec) \
                                  if hasattr(msg, 'header') else None
            self._try_send()

    def _image_cb(self, msg):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        with self._lock:
            self._latest_image = frame
            self._image_stamp  = (msg.header.stamp.sec, msg.header.stamp.nanosec)
            self._try_send()

    # ── Send logic (called inside lock) ───────────────────────────────────────

    def _try_send(self):
        """Called whenever either topic updates. Sends if both are ready and
        detections are non-empty. Clears state after sending to avoid
        double-sending the same pair."""

        if self._latest_json is None or self._latest_image is None:
            return

        # Parse JSON — skip if no detections
        try:
            data = json.loads(self._latest_json)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'JSON parse error: {e}')
            return

        if not data.get('detections'):
            # No detections this frame — clear and wait for next pair
            self._latest_json  = None
            self._latest_image = None
            return

        # Encode image as JPEG
        encode_params = [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality]
        ok, jpeg_buf = cv2.imencode('.jpg', self._latest_image, encode_params)
        if not ok:
            self.get_logger().error('Failed to JPEG-encode detection image')
            return

        json_str  = self._latest_json
        jpeg_bytes = jpeg_buf.tobytes()
        n_detections = len(data['detections'])

        # Clear state before sending (send happens in background thread)
        self._latest_json  = None
        self._latest_image = None

        # Fire-and-forget in a background thread so we don't block callbacks
        t = threading.Thread(
            target=self._post,
            args=(json_str, jpeg_bytes, n_detections),
            daemon=True,
        )
        t.start()

    # ── HTTP POST ─────────────────────────────────────────────────────────────

    def _post(self, json_str, jpeg_bytes, n_detections):
        """Runs in background thread. Builds multipart POST and sends it."""
        try:
            boundary = b'----ROS2DetectionBoundary'
            body = _build_multipart(boundary, json_str.encode(), jpeg_bytes)

            req = urllib.request.Request(
                self._url,
                data=body,
                method='POST',
            )
            req.add_header(
                'Content-Type',
                f'multipart/form-data; boundary={boundary.decode()}'
            )
            req.add_header('Content-Length', str(len(body)))

            with urllib.request.urlopen(req, timeout=self._timeout) as resp:
                status = resp.status
                self.get_logger().info(
                    f'Sent {n_detections} detection(s) → HTTP {status}'
                )

        except urllib.error.URLError as e:
            self.get_logger().warn(f'Could not reach ground station: {e.reason}')
        except Exception as e:
            self.get_logger().error(f'Unexpected send error: {e}')


# ── Multipart builder (no external dependencies) ──────────────────────────────

def _build_multipart(boundary: bytes, json_bytes: bytes, jpeg_bytes: bytes) -> bytes:
    buf = io.BytesIO()

    # JSON field
    buf.write(b'--' + boundary + b'\r\n')
    buf.write(b'Content-Disposition: form-data; name="json"\r\n')
    buf.write(b'Content-Type: application/json\r\n\r\n')
    buf.write(json_bytes)
    buf.write(b'\r\n')

    # Image field
    buf.write(b'--' + boundary + b'\r\n')
    buf.write(b'Content-Disposition: form-data; name="image"; filename="detection.jpg"\r\n')
    buf.write(b'Content-Type: image/jpeg\r\n\r\n')
    buf.write(jpeg_bytes)
    buf.write(b'\r\n')

    # Close
    buf.write(b'--' + boundary + b'--\r\n')
    return buf.getvalue()


def main(args=None):
    rclpy.init(args=args)
    node = JetsonModemNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
