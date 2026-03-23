#!/usr/bin/env python3
"""
fusion_node.py — ROS2 Sensor Fusion Node
Fuses YOLOv8 camera detections with LiDAR scan data to produce
confirmed, zone-classified tracked objects with real-world distances.

Subscribes:
  /camera/image_raw          (sensor_msgs/Image)
  /scan                      (sensor_msgs/LaserScan)

Publishes:
  /tracked_objects           (visualization_msgs/MarkerArray)  — RViz markers
  /camera_obstacles/cloud    (sensor_msgs/PointCloud2)         — Nav2 costmap input
  /detections/debug_image    (sensor_msgs/Image)               — annotated debug view

Requirements:
  pip install ultralytics opencv-python numpy
  apt install ros-<distro>-vision-msgs ros-<distro>-sensor-msgs-py
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
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
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False


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
    bearing_rad: float                   # horizontal angle from camera centre
    lidar_distance: Optional[float] = None
    estimated_distance: Optional[float] = None
    real_height: Optional[float] = None
    fusion_confidence: str = "LOW"       # "HIGH" | "MEDIUM" | "LOW"


@dataclass
class TrackedObject:
    """Confirmed fused detection ready for navigation."""
    detection: Detection
    distance: float      # best available distance (m)
    x: float             # robot-frame X (forward)
    y: float             # robot-frame Y (left)
    z: float             # robot-frame Z (up, estimated real height)
    is_safety_critical: bool = False


# ─────────────────────────────────────────────────────────────────────────────
# Zone classifier
# ─────────────────────────────────────────────────────────────────────────────

class ZoneClassifier:
    """
    Classifies detections as HIGH / MID / LOW.

    Calibrated path  — pixel row + distance → real-world height via camera geometry.
    Simple fallback  — pure pixel-Y thresholds when no distance is available.

    Tune the constants below to match your actual camera mount.
    """

    # Image resolution
    IMG_H = 720
    IMG_W = 1280

    # Camera mount (measure on your robot)
    MOUNT_HEIGHT_M = 1.0      # height from floor (m)
    TILT_DOWN_DEG  = 10.0     # downward tilt from horizontal (degrees)
    V_FOV_DEG      = 48.0     # vertical field of view (degrees)
    H_FOV_DEG      = 90.0     # horizontal field of view (degrees)

    # Real-world zone thresholds (metres)
    HIGH_THRESHOLD_M = 1.5
    LOW_THRESHOLD_M  = 0.4

    # Simple pixel thresholds (fraction of frame height, fallback only)
    SIMPLE_HIGH_FRAC = 0.30
    SIMPLE_LOW_FRAC  = 0.70

    # Known object heights for monocular distance estimation (metres)
    KNOWN_HEIGHTS = {
        "person":     1.70,
        "bottle":     0.28,
        "chair":      0.90,
        "cup":        0.12,
        "backpack":   0.50,
        "suitcase":   0.60,
        "laptop":     0.03,
        "cell phone": 0.15,
    }
    FOCAL_LENGTH_PX = 600.0   # calibrate with a checkerboard for accuracy

    # ── Public API ────────────────────────────────────────────────────────────

    def classify(self, det: Detection, lidar_distance: Optional[float] = None) -> str:
        """Return "HIGH" | "MID" | "LOW", preferring calibrated path."""
        cy = (det.bbox[1] + det.bbox[3]) / 2
        dist = lidar_distance or self._estimate_distance(det)

        if dist is not None:
            h = self._pixel_to_real_height(cy, dist)
            det.real_height = h
            return self._height_to_zone(h)

        return self._simple_zone(cy)

    def pixel_to_bearing(self, pixel_x: float) -> float:
        """Pixel X → horizontal bearing (rad). Positive = left of robot."""
        frac = pixel_x / self.IMG_W
        return math.radians((0.5 - frac) * self.H_FOV_DEG)

    # ── Internal helpers ──────────────────────────────────────────────────────

    def _pixel_to_real_height(self, pixel_y: float, distance_m: float) -> float:
        """
        Pinhole camera model:
          angle_from_centre = (pixel_y/IMG_H - 0.5) * V_FOV  [+ = below centre]
          angle_below_horiz = angle_from_centre + tilt_down
          real_height = mount_height - distance * tan(angle_below_horiz)
        """
        angle_from_centre = (pixel_y / self.IMG_H - 0.5) * self.V_FOV_DEG
        angle_below_horiz = angle_from_centre + self.TILT_DOWN_DEG
        return self.MOUNT_HEIGHT_M - distance_m * math.tan(
            math.radians(angle_below_horiz))

    def _height_to_zone(self, h: float) -> str:
        if h > self.HIGH_THRESHOLD_M: return "HIGH"
        if h < self.LOW_THRESHOLD_M:  return "LOW"
        return "MID"

    def _simple_zone(self, cy: float) -> str:
        frac = cy / self.IMG_H
        if frac < self.SIMPLE_HIGH_FRAC: return "HIGH"
        if frac > self.SIMPLE_LOW_FRAC:  return "LOW"
        return "MID"

    def _estimate_distance(self, det: Detection) -> Optional[float]:
        """Rough monocular distance estimate from known object height."""
        known_h = self.KNOWN_HEIGHTS.get(det.class_name.lower())
        if known_h is None:
            return None
        bbox_h = det.bbox[3] - det.bbox[1]
        if bbox_h <= 0:
            return None
        dist = (known_h * self.FOCAL_LENGTH_PX) / bbox_h
        det.estimated_distance = dist
        return dist


# ─────────────────────────────────────────────────────────────────────────────
# LiDAR helper
# ─────────────────────────────────────────────────────────────────────────────

class LidarHelper:
    """Converts LaserScan data into bearing-indexed range lookups."""

    BEARING_WINDOW_RAD = math.radians(3.0)   # search window around target bearing
    MAX_RANGE_M        = 10.0                 # ignore ranges beyond this

    def get_range_at_bearing(self, scan: LaserScan, bearing_rad: float) -> Optional[float]:
        """
        Return the closest valid range within ±BEARING_WINDOW_RAD of bearing.
        Returns None if no valid return (open space, out of range, or blind spot).

        Positive bearing = left of robot (ROS convention for forward-facing LiDAR).
        """
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
        """Bearing + distance + height → robot-frame (x, y, z)."""
        x = distance_m * math.cos(bearing_rad)
        y = distance_m * math.sin(bearing_rad)
        return x, y, height_m

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
    ──────────────────────────────────────────────────────────────────────────
    Zone   │ LiDAR hits?  │ Interpretation                  │ Confidence
    ───────┼──────────────┼─────────────────────────────────┼───────────
    MID    │ YES          │ Confirmed obstacle, use LiDAR d  │ HIGH
    MID    │ NO           │ Likely ghost / noise — discard   │ LOW → skip
    LOW    │ NO           │ Absence confirms below scan plane │ HIGH
    HIGH   │ NO           │ Absence confirms above scan plane │ HIGH
    LOW/HI │ YES          │ May be mid-level; use LiDAR d    │ MEDIUM
    ──────────────────────────────────────────────────────────────────────────
    """

    GHOST_CONF_THRESH = 0.50   # discard MID detections below this if LiDAR misses
    SAFETY_DISTANCE_M = 1.50   # objects closer than this are safety-critical

    def __init__(self):
        self.classifier  = ZoneClassifier()
        self.lidar_helper = LidarHelper()

    def fuse(self, detections: list, scan: LaserScan) -> list:
        tracked = []
        for det in detections:
            lidar_dist = self.lidar_helper.get_range_at_bearing(scan, det.bearing_rad)
            det.lidar_distance = lidar_dist
            det.zone = self.classifier.classify(det, lidar_dist)
            obj = self._apply_decision(det, lidar_dist)
            if obj is not None:
                tracked.append(obj)
        return tracked

    def _apply_decision(self, det: Detection, lidar_dist: Optional[float]) -> Optional[TrackedObject]:
        lidar_hit = lidar_dist is not None

        # ── Decision table ────────────────────────────────────────────────────
        if det.zone == "MID":
            if lidar_hit:
                det.fusion_confidence = "HIGH"
                best_dist = lidar_dist
            else:
                if det.confidence < self.GHOST_CONF_THRESH:
                    return None          # discard likely ghost
                det.fusion_confidence = "LOW"
                best_dist = det.estimated_distance

        else:  # LOW or HIGH
            if not lidar_hit:
                # Absence confirms out-of-plane obstacle → trust camera
                det.fusion_confidence = "HIGH"
                best_dist = det.estimated_distance
            else:
                # LiDAR sees something at same angle → use its accurate distance
                det.fusion_confidence = "MEDIUM"
                best_dist = lidar_dist

        if best_dist is None:
            return None

        # Estimate real-world height for 3D placement
        real_h = det.real_height or (
            0.1  if det.zone == "LOW"  else
            1.8  if det.zone == "HIGH" else
            self.classifier.MOUNT_HEIGHT_M
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
    TOPIC_DEBUG_IMG = "/detections/debug_image"

    MODEL_PATH        = "yolov8n.pt"   # swap to yolov8s.pt on laptop GPU
    INFERENCE_SIZE    = 320            # input size for model (smaller = faster on Pi)
    CONFIDENCE_THRESH = 0.45
    DETECTION_RATE_HZ = 10.0

    ZONE_COLORS = {"HIGH": (0, 80, 200), "MID": (200, 120, 0), "LOW": (0, 160, 60)}

    def __init__(self):
        super().__init__("fusion_node")
        self.bridge = CvBridge()
        self.fusion = FusionEngine()
        self.classifier = self.fusion.classifier
        self.latest_scan: Optional[LaserScan] = None
        self._last_infer = 0.0

        if YOLO_AVAILABLE:
            self.get_logger().info(f"Loading {self.MODEL_PATH} ...")
            self.model = YOLO(self.MODEL_PATH)
        else:
            self.model = None
            self.get_logger().warning("ultralytics not installed — running mock detections")

        # Match publisher QoS — image_raw is RELIABLE, scan is BEST_EFFORT
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
        self._publish_debug_image(frame, tracked, msg.header)

    # ── Inference ─────────────────────────────────────────────────────────────

    def _run_inference(self, frame: np.ndarray) -> list:
        if self.model is None:
            return self._mock_detections()

        results = self.model.predict(
            frame, imgsz=self.INFERENCE_SIZE,
            conf=self.CONFIDENCE_THRESH, verbose=False)

        dets = []
        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                cx = (x1 + x2) / 2
                cid   = int(box.cls[0])
                cname = self.model.names[cid]
                dets.append(Detection(
                    class_id=cid, class_name=cname,
                    confidence=float(box.conf[0]),
                    bbox=(x1, y1, x2, y2), zone="MID",
                    bearing_rad=self.classifier.pixel_to_bearing(cx)))
        return dets

    def _mock_detections(self) -> list:
        return [
            Detection(0, "person",  0.88, (560, 150, 720, 580), "MID", math.radians(-5)),
            Detection(39,"bottle",  0.72, (200, 600, 260, 710), "LOW", math.radians(20)),
        ]

    # ── Publishers ─────────────────────────────────────────────────────────────

    def _publish_markers(self, tracked: list, header: Header):
        array = MarkerArray()

        # Clear previous markers
        clear = Marker()
        clear.header = header
        clear.ns = "tracked"
        clear.action = Marker.DELETEALL
        array.markers.append(clear)

        for i, obj in enumerate(tracked):
            det  = obj.detection
            zone = det.zone

            # Cylinder at object position
            m = Marker()
            m.header = header
            m.header.frame_id = "base_link"
            m.ns = "tracked"; m.id = i
            m.type = Marker.CYLINDER; m.action = Marker.ADD
            m.pose.position.x = obj.x
            m.pose.position.y = obj.y
            m.pose.position.z = obj.z
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 0.3; m.scale.z = 0.5
            m.lifetime = Duration(sec=1)
            r, g, b = (0.9,0.2,0.1) if zone=="HIGH" else (0.1,0.8,0.2) if zone=="LOW" else (0.1,0.4,0.9)
            m.color.r=r; m.color.g=g; m.color.b=b
            m.color.a = 0.9 if obj.is_safety_critical else 0.5
            array.markers.append(m)

            # Text label
            t = Marker()
            t.header = m.header; t.ns = "labels"; t.id = 1000 + i
            t.type = Marker.TEXT_VIEW_FACING; t.action = Marker.ADD
            t.pose.position.x = obj.x
            t.pose.position.y = obj.y
            t.pose.position.z = obj.z + 0.6
            t.pose.orientation.w = 1.0
            t.scale.z = 0.18
            t.color.r = t.color.g = t.color.b = t.color.a = 1.0
            t.lifetime = Duration(sec=1)
            t.text = (f"{det.class_name} | {zone}\n"
                      f"{obj.distance:.1f}m | {det.fusion_confidence}")
            array.markers.append(t)

        self.pub_markers.publish(array)

    def _publish_pointcloud(self, tracked: list, header: Header):
        """Publish high/medium confidence detections as PointCloud2 for Nav2."""
        points = [[obj.x, obj.y, obj.z] for obj in tracked
                  if obj.detection.fusion_confidence in ("HIGH", "MEDIUM")]
        if not points:
            return
        h = Header(); h.stamp = header.stamp; h.frame_id = "base_link"
        self.pub_cloud.publish(pc2.create_cloud_xyz32(h, points))

    def _publish_debug_image(self, frame: np.ndarray, tracked: list, header: Header):
        """Annotated frame with bboxes, zones, fusion confidence, and zone bands."""
        out = frame.copy()

        # Translucent zone bands
        img_h = out.shape[0]
        overlay = out.copy()
        high_line = int(img_h * ZoneClassifier.SIMPLE_HIGH_FRAC)
        low_line  = int(img_h * ZoneClassifier.SIMPLE_LOW_FRAC)
        cv2.rectangle(overlay, (0, 0),         (out.shape[1], high_line), (0, 60, 180),  -1)
        cv2.rectangle(overlay, (0, low_line),  (out.shape[1], img_h),     (0, 140, 40),  -1)
        cv2.addWeighted(overlay, 0.12, out, 0.88, 0, out)
        cv2.putText(out, "HIGH zone", (8, high_line - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 140, 255), 1)
        cv2.putText(out, "LOW zone",  (8, low_line + 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (80, 200, 100), 1)

        # Bounding boxes
        for obj in tracked:
            det  = obj.detection
            x1, y1, x2, y2 = (int(v) for v in det.bbox)
            color     = self.ZONE_COLORS.get(det.zone, (200, 200, 200))
            thickness = 3 if obj.is_safety_critical else 1
            cv2.rectangle(out, (x1, y1), (x2, y2), color, thickness)

            label = (f"{det.class_name} {det.confidence:.2f} | "
                     f"{det.zone} | {obj.distance:.1f}m | {det.fusion_confidence}")
            (lw, lh), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.42, 1)
            cv2.rectangle(out, (x1, y1 - lh - 6), (x1 + lw, y1), color, -1)
            cv2.putText(out, label, (x1, y1 - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)

        img_msg = self.bridge.cv2_to_imgmsg(out, "bgr8")
        img_msg.header = header
        self.pub_debug.publish(img_msg)


# ─────────────────────────────────────────────────────────────────────────────
# Reactive safety node (separate, lightweight)
# ─────────────────────────────────────────────────────────────────────────────

class ReactiveSafetyNode(Node):
    """
    Watches /tracked_objects and publishes direct velocity overrides to
    /cmd_vel_safety when a confirmed obstacle is too close.

    Wire this into a twist_mux alongside Nav2's /cmd_vel output, giving
    this node higher priority so safety stops always win.
    """

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
            self.pub.publish(twist)   # zero velocity = full stop
            self.get_logger().warning(f"SAFETY STOP — obstacle at {min_dist:.2f}m")
        elif min_dist < self.SLOW_DISTANCE_M:
            twist.linear.x = 0.10
            self.pub.publish(twist)


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=3)
    fusion_node  = FusionNode()
    safety_node  = ReactiveSafetyNode()
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