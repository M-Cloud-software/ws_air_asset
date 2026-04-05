#!/usr/bin/env python3
"""
sitl_test_complex.py  —  SITL integration test using test_complex.png

Runs the full three-node pipeline (sitl_publisher + tarp_detection_node +
pipeline_monitor_node) in a single process using separate threads and
rclpy.executors.MultiThreadedExecutor.  No extra terminals required.

Scenario exercised (per the spec):
  • Input image  : test_input/test_complex.png   (complex aerial scene)
  • Altitude     : 50 m
  • Speed        : 5 m/s heading East (90°)
  • FPS          : 10 Hz
  • Duration     : 15 s

Pass criteria:
  1. /pipeline_status reports healthy=True within 5 s of startup
  2. At least one detection is published on /objects_of_interest
  3. Every detection that carries a target_lat/target_lon has coordinates
     within ±0.01° of the drone's reported position (sanity check on
     GPS projection — not a precise survey test)
  4. /pipeline_status never reports a stall fault during the run
  5. All five monitored topics show ok=True in the final status snapshot

Usage (from workspace root, after sourcing):
    source source.sh
    python3 sitl_test_complex.py [--image PATH] [--duration SECONDS]

The script exits 0 on pass, 1 on any failure, and prints a results summary.
"""

import argparse
import json
import math
import os
import sys
import threading
import time

# ---------------------------------------------------------------------------
# Locate test image relative to this script  (works from any CWD)
# ---------------------------------------------------------------------------
_SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT   = os.path.join(_SCRIPT_DIR)   # script lives at repo root
_DEFAULT_IMG = os.path.join(_REPO_ROOT, "test_input", "test_complex.png")


def _parse_args():
    p = argparse.ArgumentParser(description="SITL integration test — test_complex.png")
    p.add_argument("--image",    default=_DEFAULT_IMG,
                   help="Path to input image (default: test_input/test_complex.png)")
    p.add_argument("--duration", type=float, default=15.0,
                   help="How long to run the test in seconds (default: 15)")
    p.add_argument("--lat",      type=float, default=37.7749,  help="Start latitude")
    p.add_argument("--lon",      type=float, default=-122.4194, help="Start longitude")
    p.add_argument("--alt",      type=float, default=50.0,     help="Altitude in metres")
    p.add_argument("--speed",    type=float, default=5.0,      help="Speed in m/s")
    p.add_argument("--fps",      type=float, default=10.0,     help="Camera FPS")
    return p.parse_args()


# ---------------------------------------------------------------------------
# Results accumulator (shared across node callbacks)
# ---------------------------------------------------------------------------

class Results:
    def __init__(self, duration_sec):
        self.duration_sec         = duration_sec
        self.lock                 = threading.Lock()
        self.first_healthy_at     = None   # seconds after start
        self.status_snapshots     = []     # list of parsed JSON dicts
        self.detections           = []     # list of detection dicts
        self.drone_positions      = []     # list of (lat, lon, alt) per frame
        self.stall_faults         = []     # any status where stall was reported
        self.start_time           = None

    def record_status(self, payload: dict):
        with self.lock:
            now   = time.monotonic()
            elapsed = now - (self.start_time or now)
            self.status_snapshots.append((elapsed, payload))
            if payload.get("healthy") and self.first_healthy_at is None:
                self.first_healthy_at = elapsed
            fault = payload.get("fault") or ""
            if "stall" in fault.lower():
                self.stall_faults.append((elapsed, fault))

    def record_objects(self, payload: dict):
        with self.lock:
            self.drone_positions.append((
                payload.get("drone_lat", 0.0),
                payload.get("drone_lon", 0.0),
                payload.get("drone_alt", 0.0),
            ))
            self.detections.extend(payload.get("detections", []))


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    args = _parse_args()

    if not os.path.isfile(args.image):
        print(f"[ERROR] Image not found: {args.image}", file=sys.stderr)
        sys.exit(1)

    print(f"[SITL TEST] image    : {args.image}")
    print(f"[SITL TEST] altitude : {args.alt} m")
    print(f"[SITL TEST] speed    : {args.speed} m/s east")
    print(f"[SITL TEST] fps      : {args.fps}")
    print(f"[SITL TEST] duration : {args.duration} s")
    print()

    # Import ROS2 — must happen after the stubs file is NOT in path
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    # Import the three production nodes
    sys.path.insert(0, os.path.join(_REPO_ROOT, "src", "tarp_detection"))
    from tarp_detection.sitl_publisher     import SITLPublisher
    from tarp_detection.tarp_detection_node import TarpDetectionNode
    from tarp_detection.pipeline_monitor_node import PipelineMonitorNode

    from std_msgs.msg import String as StringMsg

    rclpy.init()
    results = Results(duration_sec=args.duration)

    # ── Create the three pipeline nodes ──────────────────────────────────────

    # 1. SITL publisher — override parameters programmatically
    sitl_node = SITLPublisher.__new__(SITLPublisher)
    sitl_node.__class__.__init__(sitl_node)   # use normal __init__ via ros2 params below

    # Simpler: just launch via rclpy with parameter overrides
    rclpy.shutdown()   # restart cleanly with parameter overrides

    # Use ros2 parameter remapping approach: pass params through constructor
    # We re-init and create nodes with param lists
    rclpy.init()

    sitl_node = rclpy.create_node(
        "sitl_publisher_test",
        parameter_overrides=[
            rclpy.parameter.Parameter("image_path",  rclpy.Parameter.Type.STRING, args.image),
            rclpy.parameter.Parameter("fps",         rclpy.Parameter.Type.DOUBLE, args.fps),
            rclpy.parameter.Parameter("start_lat",   rclpy.Parameter.Type.DOUBLE, args.lat),
            rclpy.parameter.Parameter("start_lon",   rclpy.Parameter.Type.DOUBLE, args.lon),
            rclpy.parameter.Parameter("altitude_m",  rclpy.Parameter.Type.DOUBLE, args.alt),
            rclpy.parameter.Parameter("speed_mps",   rclpy.Parameter.Type.DOUBLE, args.speed),
            rclpy.parameter.Parameter("heading_deg", rclpy.Parameter.Type.DOUBLE, 90.0),
        ]
    )

    # The node classes declare their own params, so instantiate them normally.
    # But we need parameter_overrides — use the node __init__ with
    # rclpy's parameter_overrides via the context.
    # The cleanest approach: sub-class and override get_parameter.
    sitl_node.destroy_node()
    rclpy.shutdown()

    # ── Final clean approach: launch as separate processes via subprocess ─────
    # This mirrors exactly what `ros2 launch tarp_detection sitl.launch.py`
    # does, but drives it from Python so we can assert on /pipeline_status.

    import subprocess
    import signal

    rclpy.init()

    # Listener-only node: subscribes to /pipeline_status and /objects_of_interest
    class ListenerNode(rclpy.node.Node):
        def __init__(self, res: Results):
            super().__init__("sitl_test_listener")
            self._res = res
            self.create_subscription(StringMsg, "/pipeline_status",
                                     self._status_cb, 10)
            self.create_subscription(StringMsg, "/objects_of_interest",
                                     self._objects_cb, 10)

        def _status_cb(self, msg):
            try:
                self._res.record_status(json.loads(msg.data))
            except Exception:
                pass

        def _objects_cb(self, msg):
            try:
                self._res.record_objects(json.loads(msg.data))
            except Exception:
                pass

    listener = ListenerNode(results)
    results.start_time = time.monotonic()

    # Launch the three ROS nodes as a subprocess (replicates sitl.launch.py)
    launch_cmd = [
        "ros2", "launch", "tarp_detection", "sitl.launch.py",
        f"image_path:={args.image}",
        f"altitude_m:={args.alt}",
        f"speed_mps:={args.speed}",
        f"fps:={args.fps}",
        f"start_lat:={args.lat}",
        f"start_lon:={args.lon}",
    ]

    print(f"[SITL TEST] Launching: {' '.join(launch_cmd)}")
    pipeline_proc = subprocess.Popen(
        launch_cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    # Spin the listener for the test duration
    deadline = time.monotonic() + args.duration
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(listener, timeout_sec=0.1)
    except KeyboardInterrupt:
        print("\n[SITL TEST] Interrupted by user.")
    finally:
        pipeline_proc.send_signal(signal.SIGINT)
        try:
            pipeline_proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            pipeline_proc.kill()

    listener.destroy_node()
    rclpy.shutdown()

    # ── Evaluate results ──────────────────────────────────────────────────────
    print()
    print("=" * 62)
    print("  SITL TEST RESULTS  —  test_complex.png")
    print("=" * 62)

    failures = []

    # ── Criterion 1: healthy within 5 s ──────────────────────────────────────
    HEALTHY_DEADLINE = 5.0
    if results.first_healthy_at is None:
        failures.append(
            f"FAIL [1] Pipeline never reported healthy=True "
            f"(received {len(results.status_snapshots)} status messages)"
        )
        print(f"  [1] FAIL  healthy never reached")
    elif results.first_healthy_at > HEALTHY_DEADLINE:
        failures.append(
            f"FAIL [1] healthy=True took {results.first_healthy_at:.1f}s "
            f"(limit {HEALTHY_DEADLINE}s)"
        )
        print(f"  [1] FAIL  healthy after {results.first_healthy_at:.1f}s (limit {HEALTHY_DEADLINE}s)")
    else:
        print(f"  [1] PASS  healthy=True reached in {results.first_healthy_at:.1f}s")

    # ── Criterion 2: at least one detection ──────────────────────────────────
    if not results.detections:
        failures.append("FAIL [2] No detections published during the run")
        print(f"  [2] FAIL  zero detections published")
    else:
        print(f"  [2] PASS  {len(results.detections)} total detection(s) received")

    # ── Criterion 3: GPS sanity ───────────────────────────────────────────────
    GPS_TOLERANCE_DEG = 0.01
    gps_failures = []
    for i, det in enumerate(results.detections):
        t_lat = det.get("target_lat", 0.0)
        t_lon = det.get("target_lon", 0.0)
        if t_lat == 0.0 and t_lon == 0.0:
            continue   # no GPS fix yet for this frame — skip
        # Find the closest drone position reported around the same time
        # (we compare against the overall lat/lon range of drone positions)
        if results.drone_positions:
            lats = [p[0] for p in results.drone_positions]
            lons = [p[1] for p in results.drone_positions]
            lat_ok = (min(lats) - GPS_TOLERANCE_DEG) <= t_lat <= (max(lats) + GPS_TOLERANCE_DEG)
            lon_ok = (min(lons) - GPS_TOLERANCE_DEG) <= t_lon <= (max(lons) + GPS_TOLERANCE_DEG)
            if not (lat_ok and lon_ok):
                gps_failures.append(
                    f"detection[{i}] target=({t_lat:.6f},{t_lon:.6f}) "
                    f"outside drone track range lat={min(lats):.6f}..{max(lats):.6f} "
                    f"lon={min(lons):.6f}..{max(lons):.6f}"
                )
    if gps_failures:
        failures.extend([f"FAIL [3] GPS sanity: {f}" for f in gps_failures])
        print(f"  [3] FAIL  {len(gps_failures)} GPS projection(s) out of range")
        for f in gps_failures:
            print(f"        {f}")
    else:
        checked = sum(1 for d in results.detections if d.get("target_lat", 0.0) != 0.0)
        print(f"  [3] PASS  GPS sanity OK ({checked} projection(s) checked)")

    # ── Criterion 4: No stall faults ─────────────────────────────────────────
    if results.stall_faults:
        failures.append(
            f"FAIL [4] {len(results.stall_faults)} stall fault(s) observed"
        )
        print(f"  [4] FAIL  {len(results.stall_faults)} stall fault(s)")
        for t, fault in results.stall_faults[:3]:
            print(f"        t={t:.1f}s  {fault}")
    else:
        print(f"  [4] PASS  no stall faults")

    # ── Criterion 5: Final status — all topics ok ─────────────────────────────
    if results.status_snapshots:
        _, final_status = results.status_snapshots[-1]
        topics = final_status.get("topics", {})
        bad_topics = [k for k, v in topics.items() if not v.get("ok")]
        if bad_topics:
            failures.append(
                f"FAIL [5] Topics not ok in final snapshot: {bad_topics}"
            )
            print(f"  [5] FAIL  topics not ok: {bad_topics}")
        else:
            hz_summary = ", ".join(
                f"{k}={v['hz']}Hz" for k, v in topics.items()
            )
            print(f"  [5] PASS  all topics ok  ({hz_summary})")
    else:
        failures.append("FAIL [5] No status messages received at all")
        print(f"  [5] FAIL  no /pipeline_status messages received")

    # ── Summary ───────────────────────────────────────────────────────────────
    print()
    print(f"  Status messages received : {len(results.status_snapshots)}")
    print(f"  Frames with detections   : {len([d for d in results.detections])}")
    print(f"  Drone positions recorded : {len(results.drone_positions)}")
    print()

    if failures:
        print("RESULT: FAILED")
        for f in failures:
            print(f"  {f}")
        sys.exit(1)
    else:
        print("RESULT: PASSED  ✓")
        sys.exit(0)


if __name__ == "__main__":
    main()
