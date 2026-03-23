#!/usr/bin/env python3
"""
camera_publisher_node.py — ROS2 USB camera publisher
Optimised for Logitech C270 with MJPEG capture.

Publishes:
  /camera/image_raw          (sensor_msgs/Image)
  /camera/image/compressed   (sensor_msgs/CompressedImage)

Parameters:
  device        (int,  default 0)     — /dev/videoN index
  fps           (int,  default 30)    — publish rate
  width         (int,  default 960)   — capture width  (C270 native 720p width)
  height        (int,  default 720)   — capture height
  use_mjpeg     (bool, default True)  — request MJPEG from camera (recommended)
  frame_id      (str,  default 'camera_link')
  jpeg_quality  (int,  default 80)    — re-encode quality for /compressed topic
  show_local    (bool, default False) — cv2.imshow preview (laptop only)

Usage:
  ros2 run camera_fusion camera_publisher_node
  ros2 run camera_fusion camera_publisher_node --ros-args -p show_local:=true
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2


class CameraPublisherNode(Node):

    def __init__(self):
        super().__init__("camera_publisher_node")

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter("device",       0)
        self.declare_parameter("fps",          30)
        self.declare_parameter("width",        960)
        self.declare_parameter("height",       720)
        self.declare_parameter("use_mjpeg",    True)
        self.declare_parameter("frame_id",     "camera_link")
        self.declare_parameter("jpeg_quality", 80)
        self.declare_parameter("show_local",   False)

        self.device       = self.get_parameter("device").value
        self.fps          = self.get_parameter("fps").value
        self.width        = self.get_parameter("width").value
        self.height       = self.get_parameter("height").value
        self.use_mjpeg    = self.get_parameter("use_mjpeg").value
        self.frame_id     = self.get_parameter("frame_id").value
        self.jpeg_quality = self.get_parameter("jpeg_quality").value
        self.show_local   = self.get_parameter("show_local").value

        # ── Camera setup ──────────────────────────────────────────────────────
        self.cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)

        if not self.cap.isOpened():
            self.get_logger().error(
                f"Cannot open /dev/video{self.device}. "
                f"Run 'ls /dev/video*' and set -p device:=N")
            raise RuntimeError("Camera not found")

        # IMPORTANT: set format FIRST — must come before resolution and FPS
        # otherwise the camera ignores the resolution request
        if self.use_mjpeg:
            self.cap.set(cv2.CAP_PROP_FOURCC,
                         cv2.VideoWriter_fourcc(*"MJPG"))

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS,          self.fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)     # keep only latest frame in queue
        self.cap.set(cv2.CAP_PROP_BITRATE,      5_000_000)  # 5 Mbps cap, smooths USB bursts

        # Read back actual values — camera may not honour every request
        actual_w   = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h   = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        fourcc_int = int(self.cap.get(cv2.CAP_PROP_FOURCC))
        fourcc_str = "".join(chr((fourcc_int >> (8 * i)) & 0xFF) for i in range(4))

        self.get_logger().info(
            f"Camera /dev/video{self.device} opened — "
            f"{actual_w}x{actual_h} @ {actual_fps:.0f} FPS  format: {fourcc_str}")

        if actual_w != self.width or actual_h != self.height:
            self.get_logger().warn(
                f"Requested {self.width}x{self.height} but got "
                f"{actual_w}x{actual_h} — camera does not support that mode. "
                f"Run 'v4l2-ctl --list-formats-ext' to see supported modes.")

        # ── ROS publishers ────────────────────────────────────────────────────
        self.bridge = CvBridge()

        # RELIABLE for raw — required by RViz, rqt_image_view, and most ROS2 tools
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5)

        # BEST_EFFORT for compressed — fine for WiFi streaming to laptop
        besteffort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        self.pub_raw  = self.create_publisher(
            Image,           "/camera/image_raw",        reliable_qos)
        self.pub_comp = self.create_publisher(
            CompressedImage, "/camera/image/compressed",  besteffort_qos)

        # Warmup reads — MJPEG on V4L2 often times out on the very first
        # frame while the camera initialises its buffer. Drain a few frames
        # before the ROS timer starts so the first published frame is valid.
        self.get_logger().info("Warming up camera buffer (3 frames) ...")
        for _ in range(3):
            self.cap.read()
        self.get_logger().info("Warmup done.")

        self._missed_frames = 0   # track consecutive failures

        self.timer = self.create_timer(1.0 / self.fps, self._capture_and_publish)

        self.get_logger().info(
            f"Publishing /camera/image_raw + /camera/image/compressed "
            f"at {self.fps} Hz")

    # ── Capture loop ──────────────────────────────────────────────────────────

    def _capture_and_publish(self):
        # Drain stale frames from OpenCV's internal buffer so we always
        # publish the latest frame, not one that queued up during inference.
        # grab() is cheap (no decode), retrieve() decodes only the last one.
        for _ in range(3):
            self.cap.grab()
        ret, frame = self.cap.retrieve()

        if not ret or frame is None:
            self._missed_frames += 1
            self.get_logger().warning(
                f"Frame capture failed (missed: {self._missed_frames})",
                throttle_duration_sec=3.0)
            if self._missed_frames > 30:
                self.get_logger().error(
                    "30 consecutive frame failures — check USB connection.")
                self._missed_frames = 0
            return

        self._missed_frames = 0

        stamp = self.get_clock().now().to_msg()

        # Raw image (fusion_node subscribes to this)
        raw_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        raw_msg.header.stamp    = stamp
        raw_msg.header.frame_id = self.frame_id
        self.pub_raw.publish(raw_msg)

        # Compressed image (useful when streaming to laptop over WiFi)
        encode_params = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        ok, buf = cv2.imencode(".jpg", frame, encode_params)
        if ok:
            comp_msg = CompressedImage()
            comp_msg.header.stamp    = stamp
            comp_msg.header.frame_id = self.frame_id
            comp_msg.format          = "jpeg"
            comp_msg.data            = buf.tobytes()
            self.pub_comp.publish(comp_msg)

        # Local preview — only useful on a machine with a display
        if self.show_local:
            cv2.imshow("C270 preview", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                self.get_logger().info("Preview closed.")
                rclpy.shutdown()

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def destroy_node(self):
        self.cap.release()
        if self.show_local:
            cv2.destroyAllWindows()
        super().destroy_node()


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    try:
        node = CameraPublisherNode()
        rclpy.spin(node)
    except RuntimeError:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()