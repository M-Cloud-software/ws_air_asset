#!/usr/bin/env python3
"""
pipeline_monitor_node.py

Monitors the health of the tarp detection pipeline and publishes a status
summary at 1 Hz.

Subscribes:
  /camera_feed                      (sensor_msgs/Image)
  /fmu/out/vehicle_global_position  (px4_msgs/VehicleGlobalPosition)
  /fmu/out/vehicle_local_position   (px4_msgs/VehicleLocalPosition)
  /detection_image                  (sensor_msgs/Image)
  /objects_of_interest              (std_msgs/String)

Publishes:
  /pipeline_status  (std_msgs/String)  — JSON, 1 Hz

JSON output:
  {
    "healthy": true,
    "fault": null,
    "frames_processed": 1432,
    "frames_since_last_detection": 147,
    "topics": {
      "camera_feed":         { "ok": true,  "hz": 10.2 },
      "global_position":     { "ok": true,  "hz": 4.8  },
      "local_position":      { "ok": true,  "hz": 49.1 },
      "detection_image":     { "ok": true,  "hz": 10.1 },
      "objects_of_interest": { "ok": true,  "hz": 10.1 }
    }
  }

Fault conditions (sets healthy=false and populates fault string):
  - Any topic's measured Hz drops below its configured minimum (config/monitor_params.yaml)
  - frames_processed stops incrementing for longer than stall_timeout_sec

frames_since_last_detection is informational only. It does NOT affect
the healthy flag, because no detections may simply mean no tarp is present.
"""

import json
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import String
from px4_msgs.msg import VehicleGlobalPosition, VehicleLocalPosition


class TopicWatcher:
    """
    Tracks the message rate of a single topic over a rolling 2-second window.
    Thread-safe — callbacks fire on the ROS2 executor thread, status() is
    called from the timer thread.
    """

    def __init__(self, name: str, min_hz: float):
        self.name    = name
        self.min_hz  = min_hz
        self._lock   = threading.Lock()
        self._times  = []   # timestamps of recent messages

    def tick(self):
        """Call from the topic callback each time a message arrives."""
        now = time.monotonic()
        with self._lock:
            self._times.append(now)
            # Keep only the last 2 seconds
            cutoff = now - 2.0
            self._times = [t for t in self._times if t >= cutoff]

    def status(self) -> dict:
        """
        Returns { ok: bool, hz: float }. 
        ok is False if no messages have arrived in the last 2 seconds,
        or if the measured rate is below min_hz.
        """
        now = time.monotonic()
        with self._lock:
            cutoff = now - 2.0
            recent = [t for t in self._times if t >= cutoff]

        hz = len(recent) / 2.0   # messages in last 2s / window size
        ok = hz >= self.min_hz
        return {'ok': ok, 'hz': round(hz, 1)}


class PipelineMonitorNode(Node):

    def __init__(self):
        super().__init__('pipeline_monitor')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('camera_hz_min',      5.0)
        self.declare_parameter('global_pos_hz_min',  2.0)
        self.declare_parameter('local_pos_hz_min',  15.0)
        self.declare_parameter('detection_hz_min',   5.0)
        self.declare_parameter('stall_timeout_sec',  3.0)

        self._stall_timeout = self.get_parameter('stall_timeout_sec').value

        # ── Topic watchers ────────────────────────────────────────────────────
        self._watchers = {
            'camera_feed':         TopicWatcher('camera_feed',
                                       self.get_parameter('camera_hz_min').value),
            'global_position':     TopicWatcher('global_position',
                                       self.get_parameter('global_pos_hz_min').value),
            'local_position':      TopicWatcher('local_position',
                                       self.get_parameter('local_pos_hz_min').value),
            'detection_image':     TopicWatcher('detection_image',
                                       self.get_parameter('detection_hz_min').value),
            'objects_of_interest': TopicWatcher('objects_of_interest',
                                       self.get_parameter('detection_hz_min').value),
        }

        # ── Frame counters ────────────────────────────────────────────────────
        self._lock                       = threading.Lock()
        self._frames_processed           = 0
        self._frames_since_last_det      = 0
        self._last_frame_time            = None   # monotonic time of last /objects_of_interest msg

        # ── QoS ───────────────────────────────────────────────────────────────
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(
            Image, '/camera_feed',
            lambda msg: self._watchers['camera_feed'].tick(),
            qos_best_effort)

        self.create_subscription(
            VehicleGlobalPosition, '/fmu/out/vehicle_global_position',
            lambda msg: self._watchers['global_position'].tick(),
            qos_best_effort)

        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            lambda msg: self._watchers['local_position'].tick(),
            qos_best_effort)

        self.create_subscription(
            Image, '/detection_image',
            lambda msg: self._watchers['detection_image'].tick(),
            qos_best_effort)

        self.create_subscription(
            String, '/objects_of_interest',
            self._objects_cb,
            10)

        # ── Publisher ─────────────────────────────────────────────────────────
        self._status_pub = self.create_publisher(String, '/pipeline_status', 10)

        # ── 1 Hz status timer ─────────────────────────────────────────────────
        self.create_timer(1.0, self._publish_status)

        self.get_logger().info('pipeline_monitor ready')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _objects_cb(self, msg: String):
        """
        Called every time tarp_detection_node publishes a processed frame.
        Increments frames_processed unconditionally.
        Resets frames_since_last_detection only when detections are non-empty.
        """
        self._watchers['objects_of_interest'].tick()

        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        with self._lock:
            self._frames_processed      += 1
            self._last_frame_time        = time.monotonic()

            if data.get('detections'):
                self._frames_since_last_det = 0
            else:
                self._frames_since_last_det += 1

    # ── Status publisher ──────────────────────────────────────────────────────

    def _publish_status(self):
        now = time.monotonic()

        # ── Collect topic statuses ─────────────────────────────────────────
        topic_statuses = {k: w.status() for k, w in self._watchers.items()}

        # ── Check for stalled detection node ──────────────────────────────
        with self._lock:
            frames_processed      = self._frames_processed
            frames_since_last_det = self._frames_since_last_det
            last_frame_time       = self._last_frame_time

        stalled = (
            last_frame_time is not None and
            (now - last_frame_time) > self._stall_timeout
        )

        # ── Determine overall health and fault reason ──────────────────────
        faults = []

        for name, status in topic_statuses.items():
            if not status['ok']:
                faults.append(
                    f'{name} not publishing '
                    f'(measured {status["hz"]} Hz, '
                    f'min {self._watchers[name].min_hz} Hz)'
                )

        if stalled:
            faults.append(
                f'detection node stalled — no frames processed for '
                f'>{self._stall_timeout:.0f}s'
            )

        healthy = len(faults) == 0
        fault   = '; '.join(faults) if faults else None

        # ── Build and publish payload ──────────────────────────────────────
        payload = {
            'healthy':                    healthy,
            'fault':                      fault,
            'frames_processed':           frames_processed,
            'frames_since_last_detection': frames_since_last_det,
            'topics':                     topic_statuses,
        }

        self._status_pub.publish(String(data=json.dumps(payload)))

        # Log a warning locally if unhealthy
        if not healthy:
            self.get_logger().warn(f'PIPELINE FAULT: {fault}')


def main(args=None):
    rclpy.init(args=args)
    node = PipelineMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
