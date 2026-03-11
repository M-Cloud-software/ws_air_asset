#!/usr/bin/env python3
"""
tarp_detection_node.py  –  ROS2 Humble  –  Jetson Orin Nano Super (JetPack 6.1)

Detection strategy (NO neural network / NO YOLO):
  1. Color blob detection  – Euclidean BGR or HSV distance + CCA
  2. Gradient edge detector – Sobel magnitude thresholding to find strong
                              edges that bound a colour-matched region,
                              giving sharper, less noisy bounding boxes.

Subscriptions:
  /camera_feed                          (sensor_msgs/Image)
  /fmu/out/vehicle_global_position      (px4_msgs/VehicleGlobalPosition)
  /fmu/out/vehicle_local_position       (px4_msgs/VehicleLocalPosition)

Publications:
  /objects_of_interest  (tarp_detection/DetectionArray)
  /detection_image      (sensor_msgs/Image)

GPS dead-reckoning
------------------
  VehicleGlobalPosition  → EKF-fused lat / lon / alt
  VehicleLocalPosition   → EKF vx, vy, vz  [m/s NED frame]

  vx is North (→ +lat), vy is East (→ +lon).
  Convert to deg/s:
    v_lat = vx / 111_320
    v_lon = vy / (111_320 * cos(lat))

  Propagated position:
    Δt  = (image_ROS_stamp − last_global_pos_stamp) + detection_latency
    lat += v_lat * Δt
    lon += v_lon * Δt

Pixel → GPS (nadir camera)
--------------------------
  GSD_x = 2 * alt * tan(HFOV/2) / W
  GSD_y = 2 * alt * tan(VFOV/2) / H
  dx    = (px - W/2) * GSD_x          [m East  of drone]
  dy    = (py - H/2) * GSD_y          [m South of drone]
  target_lat = drone_lat - dy / 111_320
  target_lon = drone_lon + dx / (111_320 * cos(lat))
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

from tarp_detection.msg import Detection, DetectionArray


# ─────────────────────────────────────────────────────────────────────────────
#  Pure-CV detector (no ML)
# ─────────────────────────────────────────────────────────────────────────────
class GradientColorDetector:
    """
    Two-stage detector:
      Stage 1 – Colour mask  : pixels within `tolerance` of the target colour
      Stage 2 – Gradient mask: strong Sobel edges inside or adjacent to the
                               colour mask, used to refine blob boundaries
    The two masks are combined with a morphological pipeline to produce clean
    connected components.
    """

    def __init__(
        self,
        target_color_bgr: tuple = (0, 0, 255),
        color_tolerance: int = 30,
        min_pixels: int = 500,
        max_pixels: int = 200_000,
        use_hsv: bool = True,
        gradient_threshold: int = 40,   # Sobel magnitude cutoff (0-255)
        gradient_weight: float = 0.3,   # blend weight for gradient contribution
        morph_kernel: int = 7,          # morphological close kernel size
    ):
        self.target_bgr        = np.array(target_color_bgr, dtype=np.float32)
        self.tolerance         = color_tolerance
        self.min_pixels        = min_pixels
        self.max_pixels        = max_pixels
        self.use_hsv           = use_hsv
        self.grad_thresh       = gradient_threshold
        self.grad_weight       = gradient_weight
        self.morph_kernel      = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (morph_kernel, morph_kernel))

        if use_hsv:
            pix = np.uint8([[target_color_bgr]])
            self.target_hsv = cv2.cvtColor(pix, cv2.COLOR_BGR2HSV)[0][0].astype(np.float32)

    # ── Stage 1: colour mask ──────────────────────────────────────────────
    def _color_mask(self, frame: np.ndarray) -> np.ndarray:
        if self.use_hsv:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV).astype(np.float32)
            h_diff = np.abs(hsv[:, :, 0] - self.target_hsv[0])
            h_diff = np.minimum(h_diff, 180.0 - h_diff)   # wrap-around
            s_diff = np.abs(hsv[:, :, 1] - self.target_hsv[1])
            v_diff = np.abs(hsv[:, :, 2] - self.target_hsv[2])
            diff   = np.sqrt((h_diff * 2.0) ** 2 + s_diff ** 2 + v_diff ** 2)
        else:
            f = frame.astype(np.float32)
            diff = np.sqrt(
                (f[:, :, 0] - self.target_bgr[0]) ** 2 +
                (f[:, :, 1] - self.target_bgr[1]) ** 2 +
                (f[:, :, 2] - self.target_bgr[2]) ** 2
            )
        return (diff <= self.tolerance).astype(np.uint8)

    # ── Stage 2: gradient (Sobel) mask ───────────────────────────────────
    @staticmethod
    def _gradient_mask(gray: np.ndarray, threshold: int) -> np.ndarray:
        sx = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3)
        sy = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3)
        mag = cv2.magnitude(sx, sy)
        mag = cv2.normalize(mag, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        return (mag >= threshold).astype(np.uint8)

    # ── Combined detection ────────────────────────────────────────────────
    def detect(self, frame: np.ndarray) -> tuple[list[dict], np.ndarray]:
        """
        Run both stages and return (components, debug_mask).

        Returns
        -------
        components : list of dicts with keys
                     bbox (x1,y1,x2,y2), center (cx,cy), pixel_count,
                     gradient_energy (mean Sobel mag inside blob)
        debug_mask : uint8 single-channel combined mask (for visualisation)
        """
        gray        = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        color_mask  = self._color_mask(frame)
        grad_mask   = self._gradient_mask(gray, self.grad_thresh)

        # Dilate colour mask slightly so gradient edges just outside are captured
        dilated_color = cv2.dilate(color_mask, self.morph_kernel, iterations=1)

        # Gradient contribution: edges that border or overlap colour blobs
        grad_near_color = cv2.bitwise_and(grad_mask, dilated_color)

        # Blend: combined = colour_mask OR (weighted gradient near colour)
        # Using addWeighted on uint8: clip at 1 with threshold
        combined_float = (
            color_mask.astype(np.float32)
            + self.grad_weight * grad_near_color.astype(np.float32)
        )
        combined = (combined_float >= 0.5).astype(np.uint8)

        # Morphological close: fill small holes, smooth edges
        combined = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, self.morph_kernel)

        # Connected Component Analysis
        num_labels, _, stats, centroids = cv2.connectedComponentsWithStats(
            combined, connectivity=8
        )

        # Pre-compute Sobel magnitude for energy reporting
        sx  = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3)
        sy  = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3)
        mag = cv2.magnitude(sx, sy)

        components = []
        for i in range(1, num_labels):
            px_count = stats[i, cv2.CC_STAT_AREA]
            if not (self.min_pixels <= px_count <= self.max_pixels):
                continue

            x1 = stats[i, cv2.CC_STAT_LEFT]
            y1 = stats[i, cv2.CC_STAT_TOP]
            bw = stats[i, cv2.CC_STAT_WIDTH]
            bh = stats[i, cv2.CC_STAT_HEIGHT]
            cx = int(centroids[i][0])
            cy = int(centroids[i][1])

            # Mean gradient energy inside the bounding box (quality metric)
            roi_mag = mag[y1:y1 + bh, x1:x1 + bw]
            grad_energy = float(np.mean(roi_mag))

            components.append({
                "bbox":           (x1, y1, x1 + bw - 1, y1 + bh - 1),
                "center":         (cx, cy),
                "pixel_count":    px_count,
                "gradient_energy": grad_energy,
            })

        return components, (combined * 255)


# ─────────────────────────────────────────────────────────────────────────────
#  ROS2 Node
# ─────────────────────────────────────────────────────────────────────────────
class TarpDetectionNode(Node):

    def __init__(self):
        super().__init__("tarp_detection")

        # ── Parameters ───────────────────────────────────────────────────
        self.declare_parameter("image_topic",           "/camera_feed")
        self.declare_parameter("global_pos_topic",
                               "/fmu/out/vehicle_global_position")
        self.declare_parameter("local_pos_topic",
                               "/fmu/out/vehicle_local_position")
        self.declare_parameter("target_color_bgr",      [0, 0, 255])
        self.declare_parameter("color_tolerance",       30)
        self.declare_parameter("min_pixels",            500)
        self.declare_parameter("max_pixels",            200_000)
        self.declare_parameter("use_hsv",               True)
        self.declare_parameter("gradient_threshold",    40)
        self.declare_parameter("gradient_weight",       0.3)
        self.declare_parameter("morph_kernel_size",     7)
        self.declare_parameter("camera_hfov_deg",       84.0)
        self.declare_parameter("camera_vfov_deg",       54.0)

        image_topic    = self.get_parameter("image_topic").value
        global_topic   = self.get_parameter("global_pos_topic").value
        local_topic    = self.get_parameter("local_pos_topic").value
        color_bgr      = self.get_parameter("target_color_bgr").value
        tolerance      = self.get_parameter("color_tolerance").value
        min_px         = self.get_parameter("min_pixels").value
        max_px         = self.get_parameter("max_pixels").value
        use_hsv        = self.get_parameter("use_hsv").value
        grad_thresh    = self.get_parameter("gradient_threshold").value
        grad_weight    = self.get_parameter("gradient_weight").value
        morph_k        = self.get_parameter("morph_kernel_size").value
        self.hfov      = math.radians(self.get_parameter("camera_hfov_deg").value)
        self.vfov      = math.radians(self.get_parameter("camera_vfov_deg").value)

        # ── CV detector ──────────────────────────────────────────────────
        self.detector = GradientColorDetector(
            target_color_bgr=tuple(color_bgr),
            color_tolerance=tolerance,
            min_pixels=min_px,
            max_pixels=max_px,
            use_hsv=use_hsv,
            gradient_threshold=grad_thresh,
            gradient_weight=grad_weight,
            morph_kernel=morph_k,
        )

        self.bridge = CvBridge()

        # ── PX4 EKF state ─────────────────────────────────────────────────
        # VehicleGlobalPosition: EKF-fused lat/lon/alt
        self.global_pos: VehicleGlobalPosition | None = None
        # VehicleLocalPosition: EKF vx/vy/vz in NED frame [m/s]
        # vx = North (+lat), vy = East (+lon) — no differencing needed
        self.local_pos:  VehicleLocalPosition  | None = None

        # ── QoS ──────────────────────────────────────────────────────────
        # PX4 uXRCE-DDS publishes with BEST_EFFORT reliability
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Subscribers ──────────────────────────────────────────────────
        self.create_subscription(Image,
            image_topic, self._image_cb, px4_qos)
        self.create_subscription(VehicleGlobalPosition,
            global_topic, self._global_pos_cb, px4_qos)
        self.create_subscription(VehicleLocalPosition,
            local_topic, self._local_pos_cb, px4_qos)

        # ── Publishers ───────────────────────────────────────────────────
        self.det_pub = self.create_publisher(DetectionArray, "/objects_of_interest", 10)
        self.img_pub = self.create_publisher(Image,          "/detection_image",     px4_qos)

        self.get_logger().info(
            "TarpDetectionNode ready  (gradient + colour, PX4 uXRCE-DDS, no ML)"
        )

    # ── PX4 position callbacks ─────────────────────────────────────────────
    def _global_pos_cb(self, msg: VehicleGlobalPosition):
        """EKF-fused global position (lat/lon/alt). Replaces NavSatFix."""
        self.global_pos = msg

    def _local_pos_cb(self, msg: VehicleLocalPosition):
        """EKF local position — gives us vx/vy/vz directly in NED [m/s]."""
        self.local_pos = msg

    # ── Image callback (main pipeline) ───────────────────────────────────
    def _image_cb(self, msg: Image):
        image_ts = _stamp_sec(msg.header.stamp)

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"cv_bridge: {exc}")
            return

        h, w = frame.shape[:2]

        # ── Detection ────────────────────────────────────────────────────
        t0 = time.perf_counter()
        components, debug_mask = self.detector.detect(frame)
        dt_detect = time.perf_counter() - t0
        detection_ts = time.time()

        self.get_logger().debug(
            f"{len(components)} blob(s) found  [{dt_detect * 1000:.1f} ms]"
        )

        # ── GPS dead-reckoning ────────────────────────────────────────────
        propagated = self._propagate_gps(image_ts, detection_ts)

        # ── Build annotated image ─────────────────────────────────────────
        annotated = frame.copy()

        # Overlay the binary combined mask as a semi-transparent tint
        mask_bgr = cv2.cvtColor(debug_mask, cv2.COLOR_GRAY2BGR)
        mask_bgr[:, :, 1] = 0   # keep only blue & red channels → magenta tint
        mask_bgr[:, :, 2] = 0
        annotated = cv2.addWeighted(annotated, 0.85, mask_bgr, 0.15, 0)

        # ── Build ROS message + draw bboxes ───────────────────────────────
        arr = DetectionArray()
        arr.header.stamp    = self.get_clock().now().to_msg()
        arr.header.frame_id = "map"
        arr.drone_lat = propagated[0] if propagated else 0.0
        arr.drone_lon = propagated[1] if propagated else 0.0
        arr.drone_alt = propagated[2] if propagated else 0.0

        for comp in components:
            x1, y1, x2, y2 = comp["bbox"]
            cx, cy          = comp["center"]

            det             = Detection()
            det.label       = "tarp"
            det.confidence  = float(
                min(1.0, comp["pixel_count"] / max(self.detector.min_pixels, 1))
            )
            det.bbox_x1, det.bbox_y1 = x1, y1
            det.bbox_x2, det.bbox_y2 = x2, y2
            det.pixel_cx, det.pixel_cy = cx, cy
            det.gradient_energy = comp["gradient_energy"]

            if propagated:
                lat, lon, alt = propagated
                det.target_lat, det.target_lon = _pixel_to_gps(
                    cx, cy, w, h, lat, lon, alt, self.hfov, self.vfov
                )
                gps_txt = f"GPS {det.target_lat:.6f}, {det.target_lon:.6f}"
                cv2.putText(annotated, gps_txt, (x1, y2 + 14),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 220, 0), 1)

            # Bounding box + centroid
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 230, 120), 2)
            cv2.circle(annotated,    (cx, cy), 5,          (0, 0, 255),  -1)
            info = (f"tarp  px={comp['pixel_count']}"
                    f"  gE={comp['gradient_energy']:.0f}")
            cv2.putText(annotated, info, (x1, max(y1 - 8, 0)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 230, 120), 2)

            arr.detections.append(det)

        self.det_pub.publish(arr)

        try:
            out = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            out.header = msg.header
            self.img_pub.publish(out)
        except Exception as exc:
            self.get_logger().error(f"publish image: {exc}")

    # ── GPS dead-reckoning using PX4 EKF velocity ─────────────────────────
    def _propagate_gps(self, image_ts: float, detection_ts: float):
        """
        Use VehicleGlobalPosition for base lat/lon/alt and
        VehicleLocalPosition vx/vy (NED, m/s) for propagation.

        Δt = age of global_pos at image capture  +  detection latency
        vx = North velocity  →  Δlat = vx * Δt / 111_320
        vy = East  velocity  →  Δlon = vy * Δt / (111_320 * cos(lat))
        """
        if self.global_pos is None:
            return None

        base_lat = self.global_pos.lat
        base_lon = self.global_pos.lon
        base_alt = self.global_pos.alt

        # PX4 VehicleGlobalPosition uses timestamp in microseconds
        gps_ts_sec = self.global_pos.timestamp * 1e-6

        # Total Δt: from last known position to end of detection
        dt = (image_ts - gps_ts_sec) + (detection_ts - image_ts)
        dt = max(dt, 0.0)   # guard against clock skew

        # Velocity from EKF (NED frame, m/s)  — zero if local_pos not yet received
        vx = self.local_pos.vx if self.local_pos is not None else 0.0  # North
        vy = self.local_pos.vy if self.local_pos is not None else 0.0  # East

        mlat = 111_320.0
        mlon = 111_320.0 * math.cos(math.radians(base_lat))

        prop_lat = base_lat + (vx / mlat) * dt
        prop_lon = base_lon + (vy / mlon) * dt

        return (prop_lat, prop_lon, base_alt)


# ─────────────────────────────────────────────────────────────────────────────
#  Free functions
# ─────────────────────────────────────────────────────────────────────────────
def _stamp_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def _pixel_to_gps(
    px: int, py: int, w: int, h: int,
    drone_lat: float, drone_lon: float, alt_m: float,
    hfov: float, vfov: float,
) -> tuple[float, float]:
    alt_m = max(alt_m, 1.0)
    gsd_x = 2.0 * alt_m * math.tan(hfov / 2.0) / w
    gsd_y = 2.0 * alt_m * math.tan(vfov / 2.0) / h
    dx = (px - w / 2.0) * gsd_x     # metres East
    dy = (py - h / 2.0) * gsd_y     # metres South (y↓ in image)
    mlat = 111_320.0
    mlon = 111_320.0 * math.cos(math.radians(drone_lat))
    return (drone_lat - dy / mlat, drone_lon + dx / mlon)


# ─────────────────────────────────────────────────────────────────────────────
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


if __name__ == "__main__":
    main()
