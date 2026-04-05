#!/usr/bin/env python3
"""
test_pipeline_monitor.py

Pure-Python unit tests for pipeline_monitor_node.
Tests TopicWatcher and the health/fault logic of PipelineMonitorNode
without requiring a live ROS2 runtime.

Run with:
    python3 test_pipeline_monitor.py
or inside the workspace after sourcing:
    python3 src/tarp_detection/test/test_pipeline_monitor.py

Coverage:
  TopicWatcher
    [1] reports 0 Hz and ok=False when no ticks have arrived
    [2] counts ticks correctly over a 2-second rolling window
    [3] drops ticks that fall outside the 2-second window
    [4] ok=True when measured hz >= min_hz
    [5] ok=False when measured hz < min_hz
    [6] thread-safety — concurrent ticks do not corrupt state

  PipelineMonitorNode._publish_status logic  (tested via monkey-patching)
    [7]  healthy=True, fault=None when all topics are green
    [8]  healthy=False, fault lists every failing topic
    [9]  stall detected when last_frame_time is old
    [10] stall NOT triggered when last_frame_time is None (never started)
    [11] stall NOT triggered when frames are fresh

  PipelineMonitorNode._objects_cb
    [12] increments frames_processed on every call
    [13] resets frames_since_last_detection when detections is non-empty
    [14] increments frames_since_last_detection when detections is empty
    [15] ignores invalid JSON gracefully
"""

import json
import sys
import time
import threading
import unittest
from unittest.mock import MagicMock, patch, call
import types

# ---------------------------------------------------------------------------
# Minimal ROS2 stubs so the module can be imported without a live ROS install
# ---------------------------------------------------------------------------

# Build a fake rclpy package
rclpy_stub = types.ModuleType("rclpy")
rclpy_stub.init = lambda args=None: None
rclpy_stub.spin = lambda node: None
rclpy_stub.shutdown = lambda: None

node_stub = types.ModuleType("rclpy.node")
class _FakeNode:
    def __init__(self, name):
        self._name = name
        self._params = {}
        self._logger = MagicMock()
        self._logger.info  = lambda *a, **k: None
        self._logger.warn  = lambda *a, **k: None
        self._logger.error = lambda *a, **k: None
    def get_logger(self): return self._logger
    def declare_parameter(self, name, default): self._params[name] = default
    def get_parameter(self, name):
        m = MagicMock()
        m.value = self._params[name]
        return m
    def create_subscription(self, *a, **k): return MagicMock()
    def create_publisher(self, *a, **k): return MagicMock()
    def create_timer(self, *a, **k): return MagicMock()
    def destroy_node(self): pass
node_stub.Node = _FakeNode

qos_stub = types.ModuleType("rclpy.qos")
qos_stub.QoSProfile    = MagicMock(return_value=MagicMock())
qos_stub.ReliabilityPolicy = MagicMock()
qos_stub.HistoryPolicy     = MagicMock()

sensor_msgs_stub  = types.ModuleType("sensor_msgs")
sensor_msgs_msg   = types.ModuleType("sensor_msgs.msg")
sensor_msgs_msg.Image = MagicMock
sensor_msgs_stub.msg  = sensor_msgs_msg

std_msgs_stub     = types.ModuleType("std_msgs")
std_msgs_msg      = types.ModuleType("std_msgs.msg")

class _FakeString:
    def __init__(self, data=""):
        self.data = data
std_msgs_msg.String = _FakeString
std_msgs_stub.msg   = std_msgs_msg

px4_stub     = types.ModuleType("px4_msgs")
px4_msg_stub = types.ModuleType("px4_msgs.msg")
px4_msg_stub.VehicleGlobalPosition = MagicMock
px4_msg_stub.VehicleLocalPosition  = MagicMock
px4_stub.msg = px4_msg_stub

sys.modules.setdefault("rclpy",           rclpy_stub)
sys.modules.setdefault("rclpy.node",      node_stub)
sys.modules.setdefault("rclpy.qos",       qos_stub)
sys.modules.setdefault("sensor_msgs",     sensor_msgs_stub)
sys.modules.setdefault("sensor_msgs.msg", sensor_msgs_msg)
sys.modules.setdefault("std_msgs",        std_msgs_stub)
sys.modules.setdefault("std_msgs.msg",    std_msgs_msg)
sys.modules.setdefault("px4_msgs",        px4_stub)
sys.modules.setdefault("px4_msgs.msg",    px4_msg_stub)

# Now safe to import the production module
sys.path.insert(0, "ws_air_asset-main/src/tarp_detection")
from tarp_detection.pipeline_monitor_node import TopicWatcher, PipelineMonitorNode


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_monitor(**param_overrides):
    """
    Instantiate PipelineMonitorNode with optional parameter overrides.
    Default params match monitor_params.yaml.
    """
    defaults = {
        "camera_hz_min":     5.0,
        "global_pos_hz_min": 2.0,
        "local_pos_hz_min":  15.0,
        "detection_hz_min":  5.0,
        "stall_timeout_sec": 3.0,
    }
    defaults.update(param_overrides)

    node = PipelineMonitorNode.__new__(PipelineMonitorNode)
    # Bypass __init__; set attributes manually so tests are isolated
    _FakeNode.__init__(node, "pipeline_monitor")
    node._params = defaults

    node._stall_timeout = defaults["stall_timeout_sec"]

    node._watchers = {
        "camera_feed":         TopicWatcher("camera_feed",     defaults["camera_hz_min"]),
        "global_position":     TopicWatcher("global_position", defaults["global_pos_hz_min"]),
        "local_position":      TopicWatcher("local_position",  defaults["local_pos_hz_min"]),
        "detection_image":     TopicWatcher("detection_image", defaults["detection_hz_min"]),
        "objects_of_interest": TopicWatcher("objects_of_interest", defaults["detection_hz_min"]),
    }

    node._lock                  = threading.Lock()
    node._frames_processed      = 0
    node._frames_since_last_det = 0
    node._last_frame_time       = None
    node._status_pub            = MagicMock()

    return node


def _make_objects_msg(detections=None):
    """Return a fake std_msgs/String with a valid /objects_of_interest payload."""
    payload = {
        "drone_lat": 37.7749,
        "drone_lon": -122.4194,
        "drone_alt": 30.0,
        "detections": detections if detections is not None else [],
    }
    msg = _FakeString(data=json.dumps(payload))
    return msg


# ===========================================================================
# Tests
# ===========================================================================

class TestTopicWatcher(unittest.TestCase):

    # ── [1] Zero ticks → 0 Hz, not ok ──────────────────────────────────────
    def test_1_no_ticks_reports_zero_hz(self):
        watcher = TopicWatcher("cam", min_hz=5.0)
        status = watcher.status()
        self.assertAlmostEqual(status["hz"], 0.0, places=1)
        self.assertFalse(status["ok"])

    # ── [2] Correct Hz counting ─────────────────────────────────────────────
    def test_2_counts_ticks_correctly(self):
        watcher = TopicWatcher("cam", min_hz=5.0)
        # Inject 20 ticks within the last 2 s → 10 Hz
        now = time.monotonic()
        with watcher._lock:
            watcher._times = [now - 0.1 * i for i in range(20)]
        status = watcher.status()
        self.assertAlmostEqual(status["hz"], 10.0, places=0)

    # ── [3] Old ticks are pruned ─────────────────────────────────────────────
    def test_3_old_ticks_pruned(self):
        watcher = TopicWatcher("cam", min_hz=1.0)
        now = time.monotonic()
        with watcher._lock:
            # 5 old ticks (>2 s ago) + 2 recent ticks
            watcher._times = [now - 5.0, now - 4.0, now - 3.0,
                               now - 2.5, now - 2.1,   # still outside window
                               now - 1.0, now - 0.5]    # inside window → 2 msgs / 2s = 1 Hz
        status = watcher.status()
        self.assertAlmostEqual(status["hz"], 1.0, places=0)

    # ── [4] ok=True at exactly min_hz ───────────────────────────────────────
    def test_4_ok_true_at_min_hz(self):
        watcher = TopicWatcher("cam", min_hz=5.0)
        now = time.monotonic()
        # 10 ticks → 5 Hz (10 / 2.0 window)
        with watcher._lock:
            watcher._times = [now - 0.2 * i for i in range(10)]
        status = watcher.status()
        self.assertTrue(status["ok"])

    # ── [5] ok=False below min_hz ───────────────────────────────────────────
    def test_5_ok_false_below_min_hz(self):
        watcher = TopicWatcher("cam", min_hz=5.0)
        now = time.monotonic()
        # 6 ticks in window → 3 Hz < 5 Hz
        with watcher._lock:
            watcher._times = [now - 0.3 * i for i in range(6)]
        status = watcher.status()
        self.assertFalse(status["ok"])

    # ── [6] Thread safety ───────────────────────────────────────────────────
    def test_6_concurrent_ticks_no_corruption(self):
        watcher = TopicWatcher("cam", min_hz=0.0)
        errors = []

        def spam():
            try:
                for _ in range(500):
                    watcher.tick()
            except Exception as e:
                errors.append(e)

        threads = [threading.Thread(target=spam) for _ in range(8)]
        for t in threads: t.start()
        for t in threads: t.join()

        self.assertEqual(errors, [], f"Thread error: {errors}")
        status = watcher.status()
        self.assertGreaterEqual(status["hz"], 0.0)


class TestPipelineMonitorLogic(unittest.TestCase):

    # ── [7] All topics healthy → healthy=True, fault=None ───────────────────
    def test_7_all_topics_green(self):
        node = _make_monitor()
        now = time.monotonic()

        # Inject ticks so every watcher reports above its min_hz
        for watcher in node._watchers.values():
            with watcher._lock:
                # min_hz is at most 15 → 40 msgs/2s = 20 Hz, always safe
                watcher._times = [now - 0.05 * i for i in range(40)]

        node._last_frame_time = now - 0.5   # fresh, no stall

        node._publish_status()

        published = node._status_pub.publish.call_args[0][0]
        payload = json.loads(published.data)

        self.assertTrue(payload["healthy"])
        self.assertIsNone(payload["fault"])

    # ── [8] Failing topics → healthy=False, all faults listed ───────────────
    def test_8_failing_topics_all_listed(self):
        node = _make_monitor()
        # All watchers have zero ticks → all below min_hz
        node._last_frame_time = None    # no stall added on top

        node._publish_status()

        published = node._status_pub.publish.call_args[0][0]
        payload = json.loads(published.data)

        self.assertFalse(payload["healthy"])
        self.assertIsNotNone(payload["fault"])

        for key in node._watchers:
            self.assertIn(key, payload["fault"],
                          f"Expected '{key}' to appear in fault string")

    # ── [9] Stall detected when last_frame_time is old ──────────────────────
    def test_9_stall_detected(self):
        node = _make_monitor(stall_timeout_sec=3.0)
        now = time.monotonic()

        # Make all topic watchers pass so stall is the only fault
        for watcher in node._watchers.values():
            with watcher._lock:
                watcher._times = [now - 0.05 * i for i in range(40)]

        # last frame was 10 s ago → stalled
        node._last_frame_time = now - 10.0

        node._publish_status()

        published = node._status_pub.publish.call_args[0][0]
        payload = json.loads(published.data)

        self.assertFalse(payload["healthy"])
        self.assertIn("stall", payload["fault"].lower())

    # ── [10] No stall when last_frame_time is None ───────────────────────────
    def test_10_no_stall_when_never_started(self):
        node = _make_monitor(stall_timeout_sec=3.0)
        now = time.monotonic()

        # All watchers healthy
        for watcher in node._watchers.values():
            with watcher._lock:
                watcher._times = [now - 0.05 * i for i in range(40)]

        node._last_frame_time = None   # pipeline never received a frame yet

        node._publish_status()

        published = node._status_pub.publish.call_args[0][0]
        payload = json.loads(published.data)

        # Stall should NOT be reported — pipeline hasn't started yet
        if payload["fault"]:
            self.assertNotIn("stall", payload["fault"].lower())

    # ── [11] No stall when frames are fresh ──────────────────────────────────
    def test_11_no_stall_when_frames_fresh(self):
        node = _make_monitor(stall_timeout_sec=3.0)
        now = time.monotonic()

        for watcher in node._watchers.values():
            with watcher._lock:
                watcher._times = [now - 0.05 * i for i in range(40)]

        node._last_frame_time = now - 0.2   # very recent

        node._publish_status()

        published = node._status_pub.publish.call_args[0][0]
        payload = json.loads(published.data)

        self.assertTrue(payload["healthy"])
        self.assertIsNone(payload["fault"])


class TestObjectsCb(unittest.TestCase):

    # ── [12] frames_processed increments on every call ───────────────────────
    def test_12_frames_processed_increments(self):
        node = _make_monitor()
        for i in range(5):
            node._objects_cb(_make_objects_msg(detections=[]))
        self.assertEqual(node._frames_processed, 5)

    # ── [13] frames_since_last_detection resets on detections ────────────────
    def test_13_resets_since_last_detection_on_hit(self):
        node = _make_monitor()
        # Run 3 empty frames to accumulate counter
        for _ in range(3):
            node._objects_cb(_make_objects_msg(detections=[]))
        self.assertEqual(node._frames_since_last_det, 3)

        # Now a frame WITH a detection
        detection = {
            "label": "tarp",
            "bbox": [10, 10, 50, 50],
            "pixel_center": [30, 30],
            "pixel_count": 1600,
            "target_lat": 37.7748,
            "target_lon": -122.4192,
        }
        node._objects_cb(_make_objects_msg(detections=[detection]))
        self.assertEqual(node._frames_since_last_det, 0)

    # ── [14] frames_since_last_detection increments on empty frames ──────────
    def test_14_increments_since_last_detection_on_miss(self):
        node = _make_monitor()
        for i in range(7):
            node._objects_cb(_make_objects_msg(detections=[]))
        self.assertEqual(node._frames_since_last_det, 7)

    # ── [15] Bad JSON is ignored gracefully ──────────────────────────────────
    def test_15_invalid_json_ignored(self):
        node = _make_monitor()
        bad_msg = _FakeString(data="{{not valid json}}")
        try:
            node._objects_cb(bad_msg)
        except Exception as e:
            self.fail(f"_objects_cb raised on bad JSON: {e}")
        # frames_processed should NOT have incremented (early return)
        self.assertEqual(node._frames_processed, 0)


# ---------------------------------------------------------------------------
# Additional integration-style scenario tests
# ---------------------------------------------------------------------------

class TestScenarios(unittest.TestCase):

    def test_status_payload_schema(self):
        """Published JSON always contains the required top-level keys."""
        node = _make_monitor()
        node._publish_status()
        published = node._status_pub.publish.call_args[0][0]
        payload = json.loads(published.data)

        required_keys = {
            "healthy", "fault", "frames_processed",
            "frames_since_last_detection", "topics",
        }
        self.assertEqual(required_keys, set(payload.keys()))

    def test_topics_schema(self):
        """Each entry under 'topics' has ok (bool) and hz (float)."""
        node = _make_monitor()
        node._publish_status()
        published = node._status_pub.publish.call_args[0][0]
        payload = json.loads(published.data)

        expected_keys = {
            "camera_feed", "global_position", "local_position",
            "detection_image", "objects_of_interest",
        }
        self.assertEqual(expected_keys, set(payload["topics"].keys()))
        for key, val in payload["topics"].items():
            self.assertIn("ok", val,  f"topics.{key} missing 'ok'")
            self.assertIn("hz", val,  f"topics.{key} missing 'hz'")
            self.assertIsInstance(val["ok"], bool,  f"topics.{key}.ok is not bool")
            self.assertIsInstance(val["hz"], float, f"topics.{key}.hz is not float")

    def test_partial_topic_failure(self):
        """Only the failing topic name appears in the fault string."""
        node = _make_monitor()
        now = time.monotonic()

        # All watchers pass except camera_feed
        for name, watcher in node._watchers.items():
            if name == "camera_feed":
                continue
            with watcher._lock:
                watcher._times = [now - 0.05 * i for i in range(40)]

        node._last_frame_time = now - 0.5
        node._publish_status()

        published = node._status_pub.publish.call_args[0][0]
        payload = json.loads(published.data)

        self.assertFalse(payload["healthy"])
        self.assertIn("camera_feed", payload["fault"])
        # The healthy topics should NOT appear in the fault string
        for name in ["global_position", "local_position",
                     "detection_image", "objects_of_interest"]:
            self.assertNotIn(name, payload["fault"],
                             f"Healthy topic '{name}' wrongly in fault string")

    def test_frames_processed_reflected_in_status(self):
        """frames_processed in JSON matches internal counter after _objects_cb calls."""
        node = _make_monitor()
        for _ in range(12):
            node._objects_cb(_make_objects_msg(detections=[]))

        node._publish_status()
        published = node._status_pub.publish.call_args[0][0]
        payload = json.loads(published.data)
        self.assertEqual(payload["frames_processed"], 12)

    def test_frames_since_last_detection_reflected_in_status(self):
        """frames_since_last_detection reflects detection history correctly."""
        node = _make_monitor()
        detection = {
            "label": "tarp", "bbox": [0, 0, 10, 10],
            "pixel_center": [5, 5], "pixel_count": 900,
            "target_lat": 0.0, "target_lon": 0.0,
        }
        # Detection at frame 3, then 4 empty frames
        for _ in range(3):
            node._objects_cb(_make_objects_msg(detections=[]))
        node._objects_cb(_make_objects_msg(detections=[detection]))
        for _ in range(4):
            node._objects_cb(_make_objects_msg(detections=[]))

        node._publish_status()
        published = node._status_pub.publish.call_args[0][0]
        payload = json.loads(published.data)
        self.assertEqual(payload["frames_since_last_detection"], 4)


# ===========================================================================
# Entry point
# ===========================================================================

if __name__ == "__main__":
    loader  = unittest.TestLoader()
    suite   = unittest.TestSuite()

    for cls in [
        TestTopicWatcher,
        TestPipelineMonitorLogic,
        TestObjectsCb,
        TestScenarios,
    ]:
        suite.addTests(loader.loadTestsFromTestCase(cls))

    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    sys.exit(0 if result.wasSuccessful() else 1)
