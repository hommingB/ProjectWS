#!/usr/bin/env python3
"""
fusion_node.py — ROS2 Sensor Fusion Node
Fuses YOLOv8-pose detections with LiDAR scan data.

People detection uses pose keypoints for robust distance estimation
even when only partial body is visible (low camera mount).
All other objects use bbox-based monocular estimation as fallback.

Subscribes:
  /camera/image_raw          (sensor_msgs/Image)
  /scan                      (sensor_msgs/LaserScan)

Publishes:
  /tracked_objects           (visualization_msgs/MarkerArray)
  /camera_obstacles/cloud    (sensor_msgs/PointCloud2)
  /detections/debug_image    (sensor_msgs/Image)

Requirements:
  pip install ultralytics --break-system-packages
  pip install "numpy<2" --break-system-packages
  sudo apt install ros-jazzy-cv-bridge ros-jazzy-sensor-msgs-py
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header
from sensor_msgs.msg import CompressedImage
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False


# ─────────────────────────────────────────────────────────────────────────────
# COCO pose keypoint indices
# ─────────────────────────────────────────────────────────────────────────────

KP_NOSE         = 0
KP_LEFT_EYE     = 1
KP_RIGHT_EYE    = 2
KP_LEFT_EAR     = 3
KP_RIGHT_EAR    = 4
KP_LEFT_SHOULDER  = 5
KP_RIGHT_SHOULDER = 6
KP_LEFT_HIP     = 11
KP_RIGHT_HIP    = 12
KP_LEFT_ANKLE   = 15
KP_RIGHT_ANKLE  = 16

# Real-world distances between keypoint pairs (metres)
# Used for distance estimation when only partial body is visible
KEYPOINT_PAIRS = [
    # (kp_top, kp_bottom, real_world_distance_m, name)
    (KP_NOSE,          KP_LEFT_SHOULDER,  0.30, "head-shoulder"),
    (KP_NOSE,          KP_RIGHT_SHOULDER, 0.30, "head-shoulder"),
    (KP_LEFT_SHOULDER, KP_LEFT_HIP,       0.50, "shoulder-hip"),
    (KP_RIGHT_SHOULDER,KP_RIGHT_HIP,      0.50, "shoulder-hip"),
    (KP_LEFT_SHOULDER, KP_LEFT_ANKLE,     1.40, "shoulder-ankle"),
    (KP_RIGHT_SHOULDER,KP_RIGHT_ANKLE,    1.40, "shoulder-ankle"),
    (KP_LEFT_HIP,      KP_LEFT_ANKLE,     0.90, "hip-ankle"),
    (KP_RIGHT_HIP,     KP_RIGHT_ANKLE,    0.90, "hip-ankle"),
]

# Confidence threshold for a keypoint to be considered visible
KP_CONF_THRESH = 0.4


# ─────────────────────────────────────────────────────────────────────────────
# Data structures
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class Detection:
    """Single object detection from camera."""
    class_id: int
    class_name: str
    confidence: float
    bbox: tuple                          # (x1, y1, x2, y2) pixels
    zone: str                            # "HIGH" | "MID" | "LOW"
    bearing_rad: float
    lidar_distance: Optional[float]      = None
    estimated_distance: Optional[float]  = None
    real_height: Optional[float]         = None
    fusion_confidence: str               = "LOW"
    track_id: int                        = -1
    # Pose fields — only populated for person detections
    keypoints: Optional[np.ndarray]      = None  # shape (17, 3): x, y, conf
    visible_kp_pairs: list               = field(default_factory=list)
    pose_distance: Optional[float]       = None  # best keypoint-based distance
    head_pixel_y: Optional[float]        = None  # for HIGH zone check
    ankle_pixel_y: Optional[float]       = None  # for LOW zone check


@dataclass
class TrackedObject:
    """Confirmed fused detection ready for navigation."""
    detection: Detection
    distance: float
    x: float
    y: float
    z: float
    is_safety_critical: bool = False


# ─────────────────────────────────────────────────────────────────────────────
# Pose distance estimator
# ─────────────────────────────────────────────────────────────────────────────

class PoseDistanceEstimator:
    """
    Estimates person distance from visible keypoint pairs.

    At low camera mounts, the full body is often not visible.
    By checking multiple pairs (head-shoulder, shoulder-hip, hip-ankle)
    we can estimate distance from whichever body segment IS visible.

    Priority: pairs with larger real-world span → more accurate estimate
    (longer baseline = smaller relative error from pixel uncertainty).
    """

    def __init__(self, focal_length_px: float):
        self.focal_length_px = focal_length_px

    def estimate(self, det: Detection) -> Optional[float]:
        """
        Returns best distance estimate from keypoints, or None if insufficient
        keypoints are visible. Also populates det.visible_kp_pairs for debug.
        """
        if det.keypoints is None or len(det.keypoints) < 17:
            return None

        kpts = det.keypoints  # (17, 3): x, y, confidence

        estimates = []
        det.visible_kp_pairs = []

        for kp_top, kp_bot, real_dist_m, name in KEYPOINT_PAIRS:
            conf_top = kpts[kp_top][2]
            conf_bot = kpts[kp_bot][2]

            if conf_top < KP_CONF_THRESH or conf_bot < KP_CONF_THRESH:
                continue   # keypoint not visible enough

            # Pixel distance between the two keypoints
            px_top = kpts[kp_top][1]  # y coordinate
            px_bot = kpts[kp_bot][1]
            pixel_span = abs(px_bot - px_top)

            if pixel_span < 5:
                continue   # too small, unreliable

            # dist = (real_world_span * focal_length) / pixel_span
            dist = (real_dist_m * self.focal_length_px) / pixel_span
            estimates.append((dist, real_dist_m, name))
            det.visible_kp_pairs.append(name)

        if not estimates:
            return None

        # Use the estimate from the longest visible body segment
        # (largest real_dist_m → most accurate)
        best = max(estimates, key=lambda e: e[1])
        det.pose_distance = best[0]

        # Extract head and ankle pixel Y for zone refinement
        if kpts[KP_NOSE][2] >= KP_CONF_THRESH:
            det.head_pixel_y = kpts[KP_NOSE][1]
        if kpts[KP_LEFT_ANKLE][2] >= KP_CONF_THRESH:
            det.ankle_pixel_y = kpts[KP_LEFT_ANKLE][1]
        elif kpts[KP_RIGHT_ANKLE][2] >= KP_CONF_THRESH:
            det.ankle_pixel_y = kpts[KP_RIGHT_ANKLE][1]

        return best[0]


# ─────────────────────────────────────────────────────────────────────────────
# Zone classifier
# ─────────────────────────────────────────────────────────────────────────────

class ZoneClassifier:
    """
    Classifies detections as HIGH / MID / LOW.

    For people: uses actual keypoint positions (head_y, ankle_y) when available
    — much more accurate than bbox centre at low camera mounts.

    For all objects: calibrated pinhole model when distance is known,
    simple pixel threshold as fallback.

    ── Tune these constants to match your robot ──────────────────────────────
    """

    IMG_H = 720
    IMG_W = 960            # C270 at 960×720

    # Camera mount — MEASURE on your actual robot
    MOUNT_HEIGHT_M = 0.10  # 10cm from floor
    TILT_DOWN_DEG  = 18.0  # 18° downward tilt (calibrate with script below)
    V_FOV_DEG      = 45.0  # C270 vertical FOV
    H_FOV_DEG      = 60.0  # C270 horizontal FOV
    FOCAL_LENGTH_PX = 600.0  # calibrate with checkerboard for best accuracy

    # Zone thresholds (real-world height metres)
    HIGH_THRESHOLD_M = 1.5
    LOW_THRESHOLD_M  = 0.30  # raised slightly for 10cm mount

    # Simple pixel fallback thresholds
    SIMPLE_HIGH_FRAC = 0.30
    SIMPLE_LOW_FRAC  = 0.70

    # Known heights for non-person objects (metres)
    KNOWN_HEIGHTS = {
        "bottle":     0.28,
        "chair":      0.90,
        "cup":        0.12,
        "backpack":   0.50,
        "suitcase":   0.60,
        "laptop":     0.03,
        "cell phone": 0.15,
        "dog":        0.50,
        "cat":        0.25,
        "bicycle":    1.10,
        "motorcycle": 1.20,
    }

    # ── Public API ────────────────────────────────────────────────────────────

    def classify(self, det: Detection, lidar_distance: Optional[float] = None) -> str:
        """
        Return zone string. For people, use keypoint positions directly.
        For everything else, use calibrated pixel-to-height model.
        """
        if det.class_name.lower() == "person":
            return self._classify_person(det, lidar_distance)
        return self._classify_generic(det, lidar_distance)

    def pixel_to_bearing(self, pixel_x: float) -> float:
        frac = pixel_x / self.IMG_W
        return math.radians((0.5 - frac) * self.H_FOV_DEG)

    def pixel_to_real_height(self, pixel_y: float, distance_m: float) -> float:
        angle_from_centre = (pixel_y / self.IMG_H - 0.5) * self.V_FOV_DEG
        angle_below_horiz = angle_from_centre + self.TILT_DOWN_DEG
        return self.MOUNT_HEIGHT_M - distance_m * math.tan(
            math.radians(angle_below_horiz))

    def estimate_distance(self, det: Detection) -> Optional[float]:
        """Monocular distance from known object height (non-person objects)."""
        known_h = self.KNOWN_HEIGHTS.get(det.class_name.lower())
        if known_h is None:
            return None
        bbox_h = det.bbox[3] - det.bbox[1]
        if bbox_h <= 0:
            return None
        dist = (known_h * self.FOCAL_LENGTH_PX) / bbox_h
        det.estimated_distance = dist
        return dist

    # ── Person classification ─────────────────────────────────────────────────

    def _classify_person(self, det: Detection, lidar_distance: Optional[float]) -> str:
        """
        Use keypoint positions for zone classification when available.
        This is robust to partial body visibility at low camera mounts.

        Logic:
          - Ankle visible and real height < LOW_THRESHOLD → LOW (person crouching/sitting)
          - Head visible and real height > HIGH_THRESHOLD → HIGH (unusual but possible)
          - Otherwise MID (standing person, normal case)
        """
        dist = lidar_distance or det.pose_distance or det.estimated_distance

        if dist is not None and det.ankle_pixel_y is not None:
            ankle_h = self.pixel_to_real_height(det.ankle_pixel_y, dist)
            det.real_height = ankle_h
            if ankle_h < self.LOW_THRESHOLD_M:
                return "LOW"   # person on floor / crouching very low

        if dist is not None and det.head_pixel_y is not None:
            head_h = self.pixel_to_real_height(det.head_pixel_y, dist)
            if head_h > self.HIGH_THRESHOLD_M:
                return "HIGH"  # head very high — unusual at 10cm mount

        # Default: people are MID-level obstacles
        return "MID"

    # ── Generic classification ────────────────────────────────────────────────

    def _classify_generic(self, det: Detection, lidar_distance: Optional[float]) -> str:
        cy   = (det.bbox[1] + det.bbox[3]) / 2
        dist = lidar_distance or self.estimate_distance(det)

        if dist is not None:
            h = self.pixel_to_real_height(cy, dist)
            det.real_height = h
            return self._height_to_zone(h)

        return self._simple_zone(cy)

    def _height_to_zone(self, h: float) -> str:
        if h > self.HIGH_THRESHOLD_M: return "HIGH"
        if h < self.LOW_THRESHOLD_M:  return "LOW"
        return "MID"

    def _simple_zone(self, cy: float) -> str:
        frac = cy / self.IMG_H
        if frac < self.SIMPLE_HIGH_FRAC: return "HIGH"
        if frac > self.SIMPLE_LOW_FRAC:  return "LOW"
        return "MID"


# ─────────────────────────────────────────────────────────────────────────────
# LiDAR helper
# ─────────────────────────────────────────────────────────────────────────────

class LidarHelper:
    BEARING_WINDOW_RAD = math.radians(3.0)
    MAX_RANGE_M        = 10.0

    def get_range_at_bearing(self, scan: LaserScan, bearing_rad: float) -> Optional[float]:
        if scan is None:
            return None
        best_range = None
        best_delta = float("inf")
        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r) or r <= 0.01 or r > self.MAX_RANGE_M:
                continue
            angle = scan.angle_min + i * scan.angle_increment
            delta = abs(self._angle_diff(angle, bearing_rad))
            if delta < self.BEARING_WINDOW_RAD and delta < best_delta:
                best_delta = delta
                best_range = r
        return best_range

    def bearing_to_xy(self, bearing_rad: float, distance_m: float, height_m: float):
        return (distance_m * math.cos(bearing_rad),
                distance_m * math.sin(bearing_rad),
                height_m)

    @staticmethod
    def _angle_diff(a: float, b: float) -> float:
        d = a - b
        while d >  math.pi: d -= 2 * math.pi
        while d < -math.pi: d += 2 * math.pi
        return d


# ─────────────────────────────────────────────────────────────────────────────
# Fusion engine
# ─────────────────────────────────────────────────────────────────────────────

class FusionEngine:
    """
    Decision table
    ───────────────────────────────────────────────────────────────────────────
    Zone    │ LiDAR hits? │ Interpretation                    │ Confidence
    ────────┼─────────────┼───────────────────────────────────┼───────────
    MID     │ YES         │ Confirmed, use LiDAR distance     │ HIGH
    MID     │ NO          │ Ghost / noise — discard           │ LOW → skip
    LOW/HI  │ NO          │ Absence confirms out-of-plane     │ HIGH
    LOW/HI  │ YES         │ May be mid-level, use LiDAR dist  │ MEDIUM
    ───────────────────────────────────────────────────────────────────────────

    Distance priority for people:
      1. LiDAR (most accurate — use when available)
      2. Pose keypoints (robust to partial visibility)
      3. Bbox monocular estimate (fallback, least accurate)
    """

    GHOST_CONF_THRESH = 0.30   # lowered — fusion is the quality gate
    SAFETY_DISTANCE_M = 1.50

    def __init__(self):
        self.classifier   = ZoneClassifier()
        self.lidar_helper = LidarHelper()
        self.pose_estimator = PoseDistanceEstimator(
            focal_length_px=ZoneClassifier.FOCAL_LENGTH_PX)

    def fuse(self, detections: list, scan: LaserScan) -> list:
        tracked = []
        for det in detections:
            # For people: get pose-based distance estimate first
            if det.class_name.lower() == "person":
                self.pose_estimator.estimate(det)

            lidar_dist = self.lidar_helper.get_range_at_bearing(scan, det.bearing_rad)
            det.lidar_distance = lidar_dist
            det.zone = self.classifier.classify(det, lidar_dist)

            obj = self._apply_decision(det, lidar_dist)
            if obj is not None:
                tracked.append(obj)
        return tracked

    def _apply_decision(self, det: Detection, lidar_dist: Optional[float]) -> Optional[TrackedObject]:
        lidar_hit = lidar_dist is not None
        is_person = det.class_name.lower() == "person"

        # ── Distance priority ─────────────────────────────────────────────────
        if lidar_hit:
            best_dist = lidar_dist
        elif is_person and det.pose_distance is not None:
            best_dist = det.pose_distance
        else:
            best_dist = det.estimated_distance

        # ── Decision table ────────────────────────────────────────────────────
        if det.zone == "MID":
            if lidar_hit:
                det.fusion_confidence = "HIGH"
            else:
                if det.confidence < self.GHOST_CONF_THRESH:
                    return None
                # For people with keypoint distance, trust more than unknown objects
                if is_person and det.pose_distance is not None:
                    det.fusion_confidence = "MEDIUM"
                else:
                    det.fusion_confidence = "LOW"

        else:  # LOW or HIGH
            if not lidar_hit:
                det.fusion_confidence = "HIGH"
            else:
                det.fusion_confidence = "MEDIUM"
                best_dist = lidar_dist

        if best_dist is None:
            return None

        # Real-world height for 3D marker placement
        real_h = det.real_height or (
            0.1  if det.zone == "LOW"  else
            1.8  if det.zone == "HIGH" else
            0.9  # MID default — waist height
        )

        x, y, z = self.lidar_helper.bearing_to_xy(det.bearing_rad, best_dist, real_h)
        is_critical = (best_dist < self.SAFETY_DISTANCE_M and
                       det.fusion_confidence in ("HIGH", "MEDIUM"))

        return TrackedObject(detection=det, distance=best_dist,
                             x=x, y=y, z=z, is_safety_critical=is_critical)


# ─────────────────────────────────────────────────────────────────────────────
# Main ROS2 node
# ─────────────────────────────────────────────────────────────────────────────

class FusionNode(Node):

    TOPIC_IMAGE_IN  = "/camera/image_raw"
    TOPIC_SCAN_IN   = "/scan"
    TOPIC_MARKERS   = "/tracked_objects"
    TOPIC_CLOUD     = "/camera_obstacles/cloud"
    TOPIC_DEBUG_IMG = "/detections/debug_image/compressed"

    # yolov8s-pose gives keypoints + detection in one pass
    # same compute cost as yolov8s but adds skeleton output
    MODEL_PATH        = "yolov8s-pose.pt"
    INFERENCE_SIZE    = 416
    CONFIDENCE_THRESH = 0.25
    DETECTION_RATE_HZ = 10.0
    USE_TRACKER       = True

    ZONE_COLORS = {
        "HIGH": (0,  80,  200),
        "MID":  (200, 120, 0),
        "LOW":  (0,  160,  60),
    }
    # COCO skeleton pairs for drawing (indices into 17-keypoint array)
    SKELETON = [
        (KP_NOSE, KP_LEFT_EYE), (KP_NOSE, KP_RIGHT_EYE),
        (KP_LEFT_EYE, KP_LEFT_EAR), (KP_RIGHT_EYE, KP_RIGHT_EAR),
        (KP_LEFT_EAR, KP_LEFT_SHOULDER), (KP_RIGHT_EAR, KP_RIGHT_SHOULDER),
        (KP_LEFT_SHOULDER, KP_RIGHT_SHOULDER),
        (KP_LEFT_SHOULDER, KP_LEFT_HIP), (KP_RIGHT_SHOULDER, KP_RIGHT_HIP),
        (KP_LEFT_HIP, KP_RIGHT_HIP),
        (KP_LEFT_HIP, KP_LEFT_ANKLE), (KP_RIGHT_HIP, KP_RIGHT_ANKLE),
    ]

    def __init__(self):
        super().__init__("fusion_node")
        self.bridge    = CvBridge()
        self.fusion    = FusionEngine()
        self.classifier = self.fusion.classifier
        self.latest_scan: Optional[LaserScan] = None
        self._last_infer = 0.0

        if YOLO_AVAILABLE:
            self.get_logger().info(f"Loading {self.MODEL_PATH} ...")
            self.model = YOLO(self.MODEL_PATH)
            self.get_logger().info("Model loaded.")
        else:
            self.model = None
            self.get_logger().warning("ultralytics not installed — running mock detections")

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=5)
        besteffort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(Image,     self.TOPIC_IMAGE_IN, self._cb_image, reliable_qos)
        self.create_subscription(LaserScan, self.TOPIC_SCAN_IN,  self._cb_scan,  besteffort_qos)

        self.pub_markers = self.create_publisher(MarkerArray, self.TOPIC_MARKERS,   10)
        self.pub_cloud   = self.create_publisher(PointCloud2, self.TOPIC_CLOUD,     10)
        self.pub_debug   = self.create_publisher(Image,       self.TOPIC_DEBUG_IMG, 10)

        self.get_logger().info("FusionNode ready.")

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _cb_scan(self, msg: LaserScan):
        self.latest_scan = msg

    def _cb_image(self, msg: Image):
        now = time.monotonic()
        if now - self._last_infer < 1.0 / self.DETECTION_RATE_HZ:
            return
        self._last_infer = now

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        dets  = self._run_inference(frame)

        if self.latest_scan is not None:
            tracked = self.fusion.fuse(dets, self.latest_scan)
        else:
            self.get_logger().warning(
                "Waiting for first /scan message",
                throttle_duration_sec=5.0)
            tracked = []

        self._publish_markers(tracked,    msg.header)
        self._publish_pointcloud(tracked, msg.header)
        self._publish_debug_image(frame,  tracked, msg.header)

    # ── Inference ─────────────────────────────────────────────────────────────

    def _run_inference(self, frame: np.ndarray) -> list:
        if self.model is None:
            return self._mock_detections()

        if self.USE_TRACKER:
            results = self.model.track(
                frame,
                imgsz=self.INFERENCE_SIZE,
                conf=self.CONFIDENCE_THRESH,
                tracker="bytetrack.yaml",
                persist=True,
                verbose=False,
                agnostic_nms=True)
        else:
            results = self.model.predict(
                frame,
                imgsz=self.INFERENCE_SIZE,
                conf=self.CONFIDENCE_THRESH,
                verbose=False)

        dets = []
        for r in results:
            if r.boxes is None:
                continue
            for i, box in enumerate(r.boxes):
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                cx   = (x1 + x2) / 2
                cid  = int(box.cls[0])
                name = self.model.names[cid]
                tid  = int(box.id[0]) if (self.USE_TRACKER and
                                           box.id is not None) else -1

                # Extract pose keypoints for person detections
                keypoints = None
                if name.lower() == "person" and r.keypoints is not None:
                    try:
                        kp_data = r.keypoints.data[i]   # (17, 3) tensor
                        keypoints = kp_data.cpu().numpy()
                    except (IndexError, AttributeError):
                        pass

                det = Detection(
                    class_id=cid,
                    class_name=name,
                    confidence=float(box.conf[0]),
                    bbox=(x1, y1, x2, y2),
                    zone="MID",
                    bearing_rad=self.classifier.pixel_to_bearing(cx),
                    track_id=tid,
                    keypoints=keypoints)
                dets.append(det)

        return dets

    def _mock_detections(self) -> list:
        return [
            Detection(0,  "person", 0.88, (400, 200, 560, 680), "MID", math.radians(-5), track_id=1),
            Detection(39, "bottle", 0.72, (200, 600, 260, 710), "LOW", math.radians(20)),
        ]

    # ── Publishers ─────────────────────────────────────────────────────────────

    def _publish_markers(self, tracked: list, header: Header):
        array = MarkerArray()
        clear = Marker()
        clear.header = header
        clear.ns = "tracked"
        clear.action = Marker.DELETEALL
        array.markers.append(clear)

        for i, obj in enumerate(tracked):
            det  = obj.detection
            zone = det.zone

            m = Marker()
            m.header = header
            m.header.frame_id = "base_link"
            m.ns = "tracked"; m.id = i
            m.type = Marker.CYLINDER; m.action = Marker.ADD
            m.pose.position.x = obj.x
            m.pose.position.y = obj.y
            m.pose.position.z = obj.z
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 0.3
            m.scale.z = 1.7 if det.class_name == "person" else 0.5
            m.lifetime = Duration(sec=1)
            r, g, b = ((0.9,0.2,0.1) if zone == "HIGH" else
                       (0.1,0.8,0.2) if zone == "LOW"  else
                       (0.1,0.4,0.9))
            m.color.r = r; m.color.g = g; m.color.b = b
            m.color.a = 0.9 if obj.is_safety_critical else 0.5
            array.markers.append(m)

            # Text label — show which body parts were used for distance
            t = Marker()
            t.header = m.header; t.ns = "labels"; t.id = 1000 + i
            t.type = Marker.TEXT_VIEW_FACING; t.action = Marker.ADD
            t.pose.position.x = obj.x
            t.pose.position.y = obj.y
            t.pose.position.z = obj.z + 1.0
            t.pose.orientation.w = 1.0
            t.scale.z = 0.15
            t.color.r = t.color.g = t.color.b = t.color.a = 1.0
            t.lifetime = Duration(sec=1)

            id_str   = f"#{det.track_id}" if det.track_id >= 0 else ""
            dist_src = ""
            if det.class_name == "person":
                if det.lidar_distance is not None:
                    dist_src = "lidar"
                elif det.pose_distance is not None:
                    parts = det.visible_kp_pairs[:1]
                    dist_src = parts[0] if parts else "pose"
                else:
                    dist_src = "bbox"

            t.text = (f"{det.class_name}{id_str} | {zone}\n"
                      f"{obj.distance:.1f}m | {det.fusion_confidence}"
                      + (f"\n{dist_src}" if dist_src else ""))
            array.markers.append(t)

        self.pub_markers.publish(array)

    def _publish_pointcloud(self, tracked: list, header: Header):
        points = [[obj.x, obj.y, obj.z] for obj in tracked
                  if obj.detection.fusion_confidence in ("HIGH", "MEDIUM")]
        if not points:
            return
        h = Header(); h.stamp = header.stamp; h.frame_id = "base_link"
        self.pub_cloud.publish(pc2.create_cloud_xyz32(h, points))

    def _publish_debug_image(self, frame: np.ndarray, tracked: list, header: Header):
        out = frame.copy()

        # Zone band overlays
        img_h = out.shape[0]
        overlay = out.copy()
        high_line = int(img_h * ZoneClassifier.SIMPLE_HIGH_FRAC)
        low_line  = int(img_h * ZoneClassifier.SIMPLE_LOW_FRAC)
        cv2.rectangle(overlay, (0, 0),        (out.shape[1], high_line), (0, 60, 180), -1)
        cv2.rectangle(overlay, (0, low_line), (out.shape[1], img_h),     (0, 140, 40), -1)
        cv2.addWeighted(overlay, 0.12, out, 0.88, 0, out)
        cv2.putText(out, "HIGH", (8, high_line - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 140, 255), 1)
        cv2.putText(out, "LOW",  (8, low_line + 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (80, 200, 100), 1)

        for obj in tracked:
            det   = obj.detection
            x1, y1, x2, y2 = (int(v) for v in det.bbox)
            color = self.ZONE_COLORS.get(det.zone, (180, 180, 180))
            thick = 3 if obj.is_safety_critical else 1
            cv2.rectangle(out, (x1, y1), (x2, y2), color, thick)

            # Draw pose skeleton for people
            if det.keypoints is not None:
                self._draw_skeleton(out, det.keypoints, color)

            # Label
            id_str = f"#{det.track_id} " if det.track_id >= 0 else ""
            dist_src = ""
            if det.class_name == "person":
                if det.lidar_distance: dist_src = "[L]"
                elif det.pose_distance: dist_src = "[P]"
                else: dist_src = "[B]"
            label = (f"{id_str}{det.class_name} {det.confidence:.2f}"
                     f" {dist_src}| {det.zone} | {obj.distance:.1f}m"
                     f" | {det.fusion_confidence}")
            (lw, lh), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.38, 1)
            cv2.rectangle(out, (x1, y1 - lh - 6), (x1 + lw, y1), color, -1)
            cv2.putText(out, label, (x1, y1 - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 255, 255), 1)

        encode_params = [cv2.IMWRITE_JPEG_QUALITY, 60]
        ok, buf = cv2.imencode(".jpg", out, encode_params)
        if ok:
            comp = CompressedImage()
            comp.header = header
            comp.format = "jpeg"
            comp.data   = buf.tobytes()
            self.pub_debug.publish(comp)

    def _draw_skeleton(self, frame: np.ndarray, kpts: np.ndarray, color: tuple):
        """Draw pose skeleton lines and keypoint dots."""
        for kp_a, kp_b in self.SKELETON:
            if (kpts[kp_a][2] < KP_CONF_THRESH or
                    kpts[kp_b][2] < KP_CONF_THRESH):
                continue
            pa = (int(kpts[kp_a][0]), int(kpts[kp_a][1]))
            pb = (int(kpts[kp_b][0]), int(kpts[kp_b][1]))
            cv2.line(frame, pa, pb, color, 1, cv2.LINE_AA)

        for kp in kpts:
            if kp[2] >= KP_CONF_THRESH:
                cv2.circle(frame, (int(kp[0]), int(kp[1])), 3, color, -1, cv2.LINE_AA)


# ─────────────────────────────────────────────────────────────────────────────
# Reactive safety node
# ─────────────────────────────────────────────────────────────────────────────

class ReactiveSafetyNode(Node):
    STOP_DISTANCE_M = 0.6
    SLOW_DISTANCE_M = 1.2

    def __init__(self):
        super().__init__("reactive_safety_node")
        self.create_subscription(MarkerArray, "/tracked_objects", self._cb, 10)
        self.pub = self.create_publisher(Twist, "/cmd_vel_safety", 10)
        self.get_logger().info("ReactiveSafetyNode ready.")

    def _cb(self, msg: MarkerArray):
        min_dist = float("inf")
        for mk in msg.markers:
            if mk.type != Marker.CYLINDER:
                continue
            d = math.hypot(mk.pose.position.x, mk.pose.position.y)
            min_dist = min(min_dist, d)

        twist = Twist()
        if min_dist < self.STOP_DISTANCE_M:
            self.pub.publish(twist)
            self.get_logger().warning(
                f"SAFETY STOP — obstacle at {min_dist:.2f}m",
                throttle_duration_sec=1.0)
        elif min_dist < self.SLOW_DISTANCE_M:
            twist.linear.x = 0.10
            self.pub.publish(twist)


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=3)
    fusion_node = FusionNode()
    safety_node = ReactiveSafetyNode()
    executor.add_node(fusion_node)
    executor.add_node(safety_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        fusion_node.destroy_node()
        safety_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()