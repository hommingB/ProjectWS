#!/usr/bin/env python3
"""
fusion_node.py — ROS2 Sensor Fusion Node
Alternates between two YOLO models on consecutive frames:
  - Pose model  (even frames): people detection + keypoints for distance
  - Detection model (odd frames): COCO obstacles (bottles, chairs, etc.)

This gives effective 5 FPS per task at ~10 FPS total — enough for a slow
mobile robot while keeping Pi 5 CPU under control.

Subscribes:
  /camera/image_raw           (sensor_msgs/Image)
  /scan                       (sensor_msgs/LaserScan)

Publishes:
  /tracked_objects            (visualization_msgs/MarkerArray)
  /camera_obstacles/cloud     (sensor_msgs/PointCloud2)
  /detections/debug_image/compressed  (sensor_msgs/CompressedImage)

Requirements:
  pip install ultralytics lapx --break-system-packages
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
from sensor_msgs.msg import Image, LaserScan, PointCloud2, CompressedImage
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header
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

KP_NOSE            = 0
KP_LEFT_EYE        = 1
KP_RIGHT_EYE       = 2
KP_LEFT_EAR        = 3
KP_RIGHT_EAR       = 4
KP_LEFT_SHOULDER   = 5
KP_RIGHT_SHOULDER  = 6
KP_LEFT_HIP        = 11
KP_RIGHT_HIP       = 12
KP_LEFT_ANKLE      = 15
KP_RIGHT_ANKLE     = 16

KP_CONF_THRESH = 0.40

# Keypoint pairs for distance estimation — ordered by segment length
# (longer = more accurate). Estimator tries all and picks the longest visible.
KEYPOINT_PAIRS = [
    (KP_LEFT_SHOULDER,  KP_LEFT_ANKLE,  1.40, "shoulder-ankle L"),
    (KP_RIGHT_SHOULDER, KP_RIGHT_ANKLE, 1.40, "shoulder-ankle R"),
    (KP_LEFT_HIP,       KP_LEFT_ANKLE,  0.90, "hip-ankle L"),
    (KP_RIGHT_HIP,      KP_RIGHT_ANKLE, 0.90, "hip-ankle R"),
    (KP_LEFT_SHOULDER,  KP_LEFT_HIP,    0.50, "shoulder-hip L"),
    (KP_RIGHT_SHOULDER, KP_RIGHT_HIP,   0.50, "shoulder-hip R"),
    (KP_NOSE,           KP_LEFT_SHOULDER,  0.30, "head-shoulder L"),
    (KP_NOSE,           KP_RIGHT_SHOULDER, 0.30, "head-shoulder R"),
]

# COCO skeleton pairs for debug drawing
SKELETON = [
    (KP_NOSE, KP_LEFT_EYE), (KP_NOSE, KP_RIGHT_EYE),
    (KP_LEFT_EYE, KP_LEFT_EAR), (KP_RIGHT_EYE, KP_RIGHT_EAR),
    (KP_LEFT_EAR, KP_LEFT_SHOULDER), (KP_RIGHT_EAR, KP_RIGHT_SHOULDER),
    (KP_LEFT_SHOULDER, KP_RIGHT_SHOULDER),
    (KP_LEFT_SHOULDER, KP_LEFT_HIP), (KP_RIGHT_SHOULDER, KP_RIGHT_HIP),
    (KP_LEFT_HIP, KP_RIGHT_HIP),
    (KP_LEFT_HIP, KP_LEFT_ANKLE), (KP_RIGHT_HIP, KP_RIGHT_ANKLE),
]

# COCO classes that are obstacles worth tracking (subset of 80 classes)
OBSTACLE_CLASSES = {
    "bottle", "cup", "chair", "couch", "bed", "dining table",
    "laptop", "keyboard", "cell phone", "book", "backpack",
    "suitcase", "umbrella", "handbag", "dog", "cat",
    "bicycle", "motorcycle", "car", "truck", "bus",
}


# ─────────────────────────────────────────────────────────────────────────────
# Data structures
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class Detection:
    class_id: int
    class_name: str
    confidence: float
    bbox: tuple                         # (x1, y1, x2, y2) pixels
    zone: str                           # "HIGH" | "MID" | "LOW"
    bearing_rad: float
    source: str                         # "pose" | "det" | "mock"
    lidar_distance: Optional[float]     = None
    estimated_distance: Optional[float] = None
    real_height: Optional[float]        = None
    fusion_confidence: str              = "LOW"
    track_id: int                       = -1
    keypoints: Optional[np.ndarray]     = None  # (17, 3): x, y, conf
    visible_kp_pairs: list              = field(default_factory=list)
    pose_distance: Optional[float]      = None
    head_pixel_y: Optional[float]       = None
    ankle_pixel_y: Optional[float]      = None
    dist_source: str                    = "bbox"  # "lidar"|"pose"|"bbox"


@dataclass
class TrackedObject:
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
    Estimates person distance from whichever body segment is visible.
    Robust to partial body visibility at low camera mounts (10cm).

    Uses longest visible segment for best accuracy — longer baseline
    means smaller relative error from pixel measurement uncertainty.
    """

    def __init__(self, focal_length_px: float):
        self.focal_length_px = focal_length_px

    def estimate(self, det: Detection) -> Optional[float]:
        if det.keypoints is None or len(det.keypoints) < 17:
            return None

        kpts = det.keypoints
        estimates = []
        det.visible_kp_pairs = []

        for kp_top, kp_bot, real_m, name in KEYPOINT_PAIRS:
            if (kpts[kp_top][2] < KP_CONF_THRESH or
                    kpts[kp_bot][2] < KP_CONF_THRESH):
                continue
            # Use vertical pixel span (y axis) — more stable than diagonal
            pixel_span = abs(kpts[kp_bot][1] - kpts[kp_top][1])
            if pixel_span < 8:
                continue
            dist = (real_m * self.focal_length_px) / pixel_span
            estimates.append((dist, real_m, name))
            det.visible_kp_pairs.append(name)

        if not estimates:
            return None

        # Pick estimate from longest visible segment
        best = max(estimates, key=lambda e: e[1])
        det.pose_distance = best[0]

        # Extract specific keypoints for zone classification
        if kpts[KP_NOSE][2] >= KP_CONF_THRESH:
            det.head_pixel_y = kpts[KP_NOSE][1]
        for ankle in (KP_LEFT_ANKLE, KP_RIGHT_ANKLE):
            if kpts[ankle][2] >= KP_CONF_THRESH:
                det.ankle_pixel_y = kpts[ankle][1]
                break

        return best[0]


# ─────────────────────────────────────────────────────────────────────────────
# Zone classifier
# ─────────────────────────────────────────────────────────────────────────────

class ZoneClassifier:
    """
    Classifies detections as HIGH / MID / LOW.

    People  → keypoint-based (ankle/head position) when available
    Objects → calibrated pinhole model, pixel threshold fallback

    ── Tune these for your robot ─────────────────────────────────────────────
    Run the calibration script in the README to get FOCAL_LENGTH_PX.
    Measure MOUNT_HEIGHT_M and TILT_DOWN_DEG physically on your robot.
    """

    IMG_H = 720
    IMG_W = 960             # C270 at 960×720

    MOUNT_HEIGHT_M  = 0.10  # camera height from floor (metres)
    TILT_DOWN_DEG   = 18.0  # downward tilt from horizontal (degrees)
    V_FOV_DEG       = 45.0  # C270 vertical FOV
    H_FOV_DEG       = 60.0  # C270 horizontal FOV
    FOCAL_LENGTH_PX = 600.0 # ← CALIBRATE THIS (see README)

    HIGH_THRESHOLD_M  = 1.5
    LOW_THRESHOLD_M   = 0.30

    SIMPLE_HIGH_FRAC  = 0.30
    SIMPLE_LOW_FRAC   = 0.70

    KNOWN_HEIGHTS = {
        "bottle":     0.28,
        "cup":        0.12,
        "chair":      0.90,
        "couch":      0.85,
        "backpack":   0.50,
        "suitcase":   0.60,
        "laptop":     0.03,
        "cell phone": 0.15,
        "dog":        0.50,
        "cat":        0.25,
        "bicycle":    1.10,
        "motorcycle": 1.20,
        "car":        1.50,
    }

    def classify(self, det: Detection, lidar_dist: Optional[float] = None) -> str:
        if det.class_name.lower() == "person":
            return self._classify_person(det, lidar_dist)
        return self._classify_generic(det, lidar_dist)

    def pixel_to_bearing(self, pixel_x: float) -> float:
        return math.radians((0.5 - pixel_x / self.IMG_W) * self.H_FOV_DEG)

    def pixel_to_real_height(self, pixel_y: float, dist_m: float) -> float:
        angle = (pixel_y / self.IMG_H - 0.5) * self.V_FOV_DEG + self.TILT_DOWN_DEG
        return self.MOUNT_HEIGHT_M - dist_m * math.tan(math.radians(angle))

    def estimate_distance(self, det: Detection) -> Optional[float]:
        known_h = self.KNOWN_HEIGHTS.get(det.class_name.lower())
        if known_h is None:
            return None
        bbox_h = det.bbox[3] - det.bbox[1]
        if bbox_h <= 0:
            return None
        dist = (known_h * self.FOCAL_LENGTH_PX) / bbox_h
        det.estimated_distance = dist
        return dist

    def _classify_person(self, det: Detection, lidar_dist: Optional[float]) -> str:
        dist = lidar_dist or det.pose_distance or det.estimated_distance
        if dist is not None:
            if det.ankle_pixel_y is not None:
                ankle_h = self.pixel_to_real_height(det.ankle_pixel_y, dist)
                det.real_height = ankle_h
                if ankle_h < self.LOW_THRESHOLD_M:
                    return "LOW"
            if det.head_pixel_y is not None:
                head_h = self.pixel_to_real_height(det.head_pixel_y, dist)
                if head_h > self.HIGH_THRESHOLD_M:
                    return "HIGH"
        return "MID"

    def _classify_generic(self, det: Detection, lidar_dist: Optional[float]) -> str:
        dist = lidar_dist or self.estimate_distance(det)
        cy = (det.bbox[1] + det.bbox[3]) / 2
        if dist is not None:
            h = self.pixel_to_real_height(cy, dist)
            det.real_height = h
            if h > self.HIGH_THRESHOLD_M: return "HIGH"
            if h < self.LOW_THRESHOLD_M:  return "LOW"
            return "MID"
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

    def get_range_at_bearing(self, scan: LaserScan,
                              bearing_rad: float) -> Optional[float]:
        if scan is None:
            return None
        best, best_d = None, float("inf")
        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r) or r <= 0.01 or r > self.MAX_RANGE_M:
                continue
            angle = scan.angle_min + i * scan.angle_increment
            d = abs(self._wrap(angle - bearing_rad))
            if d < self.BEARING_WINDOW_RAD and d < best_d:
                best_d, best = d, r
        return best

    def bearing_to_xy(self, bearing: float, dist: float, height: float):
        return dist * math.cos(bearing), dist * math.sin(bearing), height

    @staticmethod
    def _wrap(a: float) -> float:
        while a >  math.pi: a -= 2 * math.pi
        while a < -math.pi: a += 2 * math.pi
        return a


# ─────────────────────────────────────────────────────────────────────────────
# Fusion engine
# ─────────────────────────────────────────────────────────────────────────────

class FusionEngine:
    """
    Decision table
    ───────────────────────────────────────────────────────────────────────────
    Zone   │ LiDAR │ Interpretation                        │ Confidence
    ───────┼───────┼───────────────────────────────────────┼───────────
    MID    │ YES   │ Confirmed — use LiDAR distance         │ HIGH
    MID    │ NO    │ Person+pose → MEDIUM, else discard     │ MEDIUM/skip
    LOW/HI │ NO    │ Absence confirms out-of-plane          │ HIGH
    LOW/HI │ YES   │ May be mid-level — use LiDAR distance  │ MEDIUM

    Distance priority:
      1. LiDAR (always preferred — most accurate)
      2. Pose keypoints (people only, robust to partial body)
      3. Bbox monocular (fallback, depends on focal calibration)
    """

    GHOST_CONF_THRESH = 0.30
    SAFETY_DISTANCE_M = 1.50

    def __init__(self):
        self.classifier    = ZoneClassifier()
        self.lidar_helper  = LidarHelper()
        self.pose_estimator = PoseDistanceEstimator(
            ZoneClassifier.FOCAL_LENGTH_PX)

    def fuse(self, detections: list, scan: LaserScan) -> list:
        tracked = []
        for det in detections:
            if det.class_name.lower() == "person":
                self.pose_estimator.estimate(det)
            lidar_dist = self.lidar_helper.get_range_at_bearing(
                scan, det.bearing_rad)
            det.lidar_distance = lidar_dist
            det.zone = self.classifier.classify(det, lidar_dist)
            obj = self._decide(det, lidar_dist)
            if obj is not None:
                tracked.append(obj)
        return tracked

    def _decide(self, det: Detection,
                lidar_dist: Optional[float]) -> Optional[TrackedObject]:
        lidar_hit = lidar_dist is not None
        is_person = det.class_name.lower() == "person"

        # ── Best distance ─────────────────────────────────────────────────────
        if lidar_hit:
            best_dist = lidar_dist
            det.dist_source = "lidar"
        elif is_person and det.pose_distance is not None:
            best_dist = det.pose_distance
            det.dist_source = "pose"
        elif det.estimated_distance is not None:
            best_dist = det.estimated_distance
            det.dist_source = "bbox"
        else:
            best_dist = None
            det.dist_source = "none"

        # ── Confidence decision ───────────────────────────────────────────────
        if det.zone == "MID":
            if lidar_hit:
                det.fusion_confidence = "HIGH"
            elif is_person and det.pose_distance is not None:
                det.fusion_confidence = "MEDIUM"
            elif det.confidence >= self.GHOST_CONF_THRESH:
                det.fusion_confidence = "LOW"
            else:
                return None   # discard ghost

        else:   # LOW or HIGH
            if not lidar_hit:
                det.fusion_confidence = "HIGH"
            else:
                det.fusion_confidence = "MEDIUM"
                best_dist = lidar_dist
                det.dist_source = "lidar"

        if best_dist is None:
            return None

        real_h = det.real_height or (
            0.10 if det.zone == "LOW"  else
            1.80 if det.zone == "HIGH" else
            0.90)

        x, y, z = self.lidar_helper.bearing_to_xy(
            det.bearing_rad, best_dist, real_h)
        critical = (best_dist < self.SAFETY_DISTANCE_M and
                    det.fusion_confidence in ("HIGH", "MEDIUM"))

        return TrackedObject(detection=det, distance=best_dist,
                             x=x, y=y, z=z, is_safety_critical=critical)


# ─────────────────────────────────────────────────────────────────────────────
# Main ROS2 node
# ─────────────────────────────────────────────────────────────────────────────

class FusionNode(Node):

    TOPIC_IMAGE_IN   = "/camera/image_raw"
    TOPIC_SCAN_IN    = "/scan"
    TOPIC_MARKERS    = "/tracked_objects"
    TOPIC_CLOUD      = "/camera_obstacles/cloud"
    TOPIC_DEBUG_COMP = "/detections/debug_image/compressed"

    # Two models — alternated on consecutive frames
    # Pose model  → people detection + keypoints (even frames)
    # Det model   → all COCO obstacles           (odd frames)
    MODEL_POSE     = "yolov8n-pose.pt"   # nano pose — ~10 FPS on Pi 5
    MODEL_DET      = "yolov8n.pt"        # nano det  — ~12 FPS on Pi 5
    INFERENCE_SIZE = 320                 # keep small for Pi 5
    CONF_POSE      = 0.30                # slightly lower — keypoints help filter
    CONF_DET       = 0.35                # slightly higher — avoid junk detections
    DETECTION_RATE_HZ = 10.0
    USE_TRACKER    = True
    DEBUG_JPEG_Q   = 55                  # compressed debug image quality (0-100)

    ZONE_COLORS = {
        "HIGH": (0,  80,  200),
        "MID":  (200, 120,  0),
        "LOW":  (0,  160,  60),
    }
    DIST_SRC_COLORS = {
        "lidar": (0,  220, 255),   # cyan
        "pose":  (180, 0,  255),   # purple
        "bbox":  (160, 160, 160),  # gray
        "none":  (80,  80,  80),
    }

    def __init__(self):
        super().__init__("fusion_node")
        self.bridge     = CvBridge()
        self.fusion     = FusionEngine()
        self.classifier = self.fusion.classifier
        self.latest_scan: Optional[LaserScan] = None
        self._last_infer  = 0.0
        self._frame_count = 0

        # Last detections from each model — merged each cycle
        self._last_pose_dets: list = []
        self._last_det_dets:  list = []

        if YOLO_AVAILABLE:
            self.get_logger().info(f"Loading {self.MODEL_POSE} ...")
            self.model_pose = YOLO(self.MODEL_POSE)
            self.get_logger().info(f"Loading {self.MODEL_DET} ...")
            self.model_det  = YOLO(self.MODEL_DET)
            self.get_logger().info("Both models loaded.")
        else:
            self.model_pose = None
            self.model_det  = None
            self.get_logger().warning("ultralytics not installed — mock mode")

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=5)
        besteffort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(
            Image, self.TOPIC_IMAGE_IN, self._cb_image, reliable_qos)
        self.create_subscription(
            LaserScan, self.TOPIC_SCAN_IN, self._cb_scan, besteffort_qos)

        self.pub_markers = self.create_publisher(
            MarkerArray,     self.TOPIC_MARKERS,    10)
        self.pub_cloud   = self.create_publisher(
            PointCloud2,     self.TOPIC_CLOUD,      10)
        self.pub_debug   = self.create_publisher(
            CompressedImage, self.TOPIC_DEBUG_COMP, 10)

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
        self._frame_count += 1

        # ── Alternate models ──────────────────────────────────────────────────
        if self._frame_count % 2 == 0:
            self._last_pose_dets = self._run_pose(frame)
        else:
            self._last_det_dets  = self._run_detection(frame)

        # Merge: pose detections (people) + det detections (obstacles)
        # People from pose model take priority over any person from det model
        obstacle_dets = [d for d in self._last_det_dets
                         if d.class_name.lower() != "person"]
        all_dets = self._last_pose_dets + obstacle_dets

        if self.latest_scan is not None:
            tracked = self.fusion.fuse(all_dets, self.latest_scan)
        else:
            self.get_logger().warning(
                "Waiting for /scan", throttle_duration_sec=5.0)
            tracked = []

        self._publish_markers(tracked,    msg.header)
        self._publish_pointcloud(tracked, msg.header)
        self._publish_debug_image(frame,  tracked, msg.header)

    # ── Inference ─────────────────────────────────────────────────────────────

    def _run_pose(self, frame: np.ndarray) -> list:
        """Run pose model — returns people detections with keypoints."""
        if self.model_pose is None:
            return [Detection(0, "person", 0.88,
                              (400, 200, 560, 680), "MID",
                              math.radians(-5), source="mock", track_id=1)]

        if self.USE_TRACKER:
            results = self.model_pose.track(
                frame, imgsz=self.INFERENCE_SIZE,
                conf=self.CONF_POSE, tracker="botsort.yaml",
                persist=True, verbose=False)
        else:
            results = self.model_pose.predict(
                frame, imgsz=self.INFERENCE_SIZE,
                conf=self.CONF_POSE, verbose=False)

        dets = []
        for r in results:
            if r.boxes is None:
                continue
            for i, box in enumerate(r.boxes):
                cid  = int(box.cls[0])
                name = self.model_pose.names[cid]
                if name.lower() != "person":
                    continue   # pose model used for people only

                x1, y1, x2, y2 = box.xyxy[0].tolist()
                cx  = (x1 + x2) / 2
                tid = int(box.id[0]) if (self.USE_TRACKER and
                                          box.id is not None) else -1

                kpts = None
                if r.keypoints is not None:
                    try:
                        kpts = r.keypoints.data[i].cpu().numpy()
                    except (IndexError, AttributeError):
                        pass

                dets.append(Detection(
                    class_id=cid, class_name=name,
                    confidence=float(box.conf[0]),
                    bbox=(x1, y1, x2, y2), zone="MID",
                    bearing_rad=self.classifier.pixel_to_bearing(cx),
                    source="pose", track_id=tid, keypoints=kpts))
        return dets

    def _run_detection(self, frame: np.ndarray) -> list:
        """Run detection model — returns obstacle detections (no keypoints)."""
        if self.model_det is None:
            return [Detection(39, "bottle", 0.72,
                              (200, 600, 260, 710), "LOW",
                              math.radians(20), source="mock")]

        if self.USE_TRACKER:
            results = self.model_det.track(
                frame, imgsz=self.INFERENCE_SIZE,
                conf=self.CONF_DET, tracker="botsort.yaml",
                persist=True, verbose=False)
        else:
            results = self.model_det.predict(
                frame, imgsz=self.INFERENCE_SIZE,
                conf=self.CONF_DET, verbose=False)

        dets = []
        for r in results:
            if r.boxes is None:
                continue
            for box in r.boxes:
                cid  = int(box.cls[0])
                name = self.model_det.names[cid]

                # Only keep obstacle-relevant classes
                if name.lower() not in OBSTACLE_CLASSES:
                    continue

                x1, y1, x2, y2 = box.xyxy[0].tolist()
                cx  = (x1 + x2) / 2
                tid = int(box.id[0]) if (self.USE_TRACKER and
                                          box.id is not None) else -1

                dets.append(Detection(
                    class_id=cid, class_name=name,
                    confidence=float(box.conf[0]),
                    bbox=(x1, y1, x2, y2), zone="MID",
                    bearing_rad=self.classifier.pixel_to_bearing(cx),
                    source="det", track_id=tid))
        return dets

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
            r, g, b = ((0.9, 0.2, 0.1) if zone == "HIGH" else
                       (0.1, 0.8, 0.2) if zone == "LOW"  else
                       (0.1, 0.4, 0.9))
            m.color.r = r; m.color.g = g; m.color.b = b
            m.color.a = 0.9 if obj.is_safety_critical else 0.5
            array.markers.append(m)

            # Label shows distance source so you can debug accuracy
            t = Marker()
            t.header = m.header; t.ns = "labels"; t.id = 1000 + i
            t.type = Marker.TEXT_VIEW_FACING; t.action = Marker.ADD
            t.pose.position.x = obj.x
            t.pose.position.y = obj.y
            t.pose.position.z = obj.z + 1.1
            t.pose.orientation.w = 1.0
            t.scale.z = 0.14
            t.color.r = t.color.g = t.color.b = t.color.a = 1.0
            t.lifetime = Duration(sec=1)
            id_str = f"#{det.track_id}" if det.track_id >= 0 else ""
            kp_str = (f"\n{det.visible_kp_pairs[0]}"
                      if det.visible_kp_pairs else "")
            t.text = (f"{det.class_name}{id_str} | {zone}\n"
                      f"{obj.distance:.2f}m [{det.dist_source}]"
                      f" {det.fusion_confidence}{kp_str}")
            array.markers.append(t)

        self.pub_markers.publish(array)

    def _publish_pointcloud(self, tracked: list, header: Header):
        points = [[obj.x, obj.y, obj.z] for obj in tracked
                  if obj.detection.fusion_confidence in ("HIGH", "MEDIUM")]
        if not points:
            return
        h = Header(); h.stamp = header.stamp; h.frame_id = "base_link"
        self.pub_cloud.publish(pc2.create_cloud_xyz32(h, points))

    def _publish_debug_image(self, frame: np.ndarray,
                              tracked: list, header: Header):
        """Publish JPEG-compressed annotated frame — low bandwidth for WSL2."""
        out = frame.copy()

        # Zone bands
        h_img = out.shape[0]
        overlay = out.copy()
        hl = int(h_img * ZoneClassifier.SIMPLE_HIGH_FRAC)
        ll = int(h_img * ZoneClassifier.SIMPLE_LOW_FRAC)
        cv2.rectangle(overlay, (0, 0),      (out.shape[1], hl), (0, 60, 180), -1)
        cv2.rectangle(overlay, (0, ll),     (out.shape[1], h_img), (0, 140, 40), -1)
        cv2.addWeighted(overlay, 0.12, out, 0.88, 0, out)
        cv2.putText(out, "HIGH", (8, hl - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 140, 255), 1)
        cv2.putText(out, "LOW",  (8, ll + 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (80, 200, 100),  1)

        # Which model ran this frame
        model_label = ("POSE" if self._frame_count % 2 == 0 else "DET")
        cv2.putText(out, f"[{model_label}] frame {self._frame_count}",
                    (out.shape[1] - 180, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

        for obj in tracked:
            det   = obj.detection
            x1, y1, x2, y2 = (int(v) for v in det.bbox)
            zone_color = self.ZONE_COLORS.get(det.zone, (180, 180, 180))
            src_color  = self.DIST_SRC_COLORS.get(det.dist_source, (160, 160, 160))
            thick = 3 if obj.is_safety_critical else 1

            # Outer box: zone colour
            cv2.rectangle(out, (x1, y1), (x2, y2), zone_color, thick)
            # Inner thin box: distance source colour
            cv2.rectangle(out, (x1+2, y1+2), (x2-2, y2-2), src_color, 1)

            # Skeleton for people
            if det.keypoints is not None:
                self._draw_skeleton(out, det.keypoints, src_color)

            # Label
            id_str = f"#{det.track_id} " if det.track_id >= 0 else ""
            src_tag = {"lidar":"[L]","pose":"[P]","bbox":"[B]","none":"[?]"
                       }.get(det.dist_source, "")
            label = (f"{id_str}{det.class_name} {det.confidence:.2f}"
                     f" {src_tag} {det.zone} {obj.distance:.1f}m"
                     f" {det.fusion_confidence}")
            (lw, lh), _ = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.38, 1)
            cv2.rectangle(out, (x1, y1 - lh - 6), (x1 + lw, y1),
                          zone_color, -1)
            cv2.putText(out, label, (x1, y1 - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 255, 255), 1)

        # Encode as JPEG — much smaller than raw for WSL2 network
        ok, buf = cv2.imencode(
            ".jpg", out, [cv2.IMWRITE_JPEG_QUALITY, self.DEBUG_JPEG_Q])
        if ok:
            comp = CompressedImage()
            comp.header = header
            comp.format = "jpeg"
            comp.data   = buf.tobytes()
            self.pub_debug.publish(comp)

    def _draw_skeleton(self, frame: np.ndarray,
                        kpts: np.ndarray, color: tuple):
        for kp_a, kp_b in SKELETON:
            if (kpts[kp_a][2] < KP_CONF_THRESH or
                    kpts[kp_b][2] < KP_CONF_THRESH):
                continue
            pa = (int(kpts[kp_a][0]), int(kpts[kp_a][1]))
            pb = (int(kpts[kp_b][0]), int(kpts[kp_b][1]))
            cv2.line(frame, pa, pb, color, 1, cv2.LINE_AA)
        for kp in kpts:
            if kp[2] >= KP_CONF_THRESH:
                cv2.circle(frame, (int(kp[0]), int(kp[1])),
                           3, color, -1, cv2.LINE_AA)


# ─────────────────────────────────────────────────────────────────────────────
# Reactive safety node
# ─────────────────────────────────────────────────────────────────────────────

class ReactiveSafetyNode(Node):
    STOP_DISTANCE_M = 0.60
    SLOW_DISTANCE_M = 1.20

    def __init__(self):
        super().__init__("reactive_safety_node")
        self.create_subscription(
            MarkerArray, "/tracked_objects", self._cb, 10)
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
                f"SAFETY STOP — {min_dist:.2f}m",
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