#!/usr/bin/env python3
"""
camera_info_publisher.py — Publishes a CameraInfo message that is timestamp-
matched to /camera/image_raw so that RTAB-Map's ApproximateTime filter can
synchronize them.

This node must be launched BEFORE rtabmap so the camera_info topic
exists when rtabmap starts.

*** HOW TO GET PROPER CALIBRATION ***
The parameters below are reasonable approximations for the Logitech C270
at 960x720. For best loop-closure results, calibrate the camera using:

    ros2 run camera_calibration cameracalibrator \
        --size 8x6 --square 0.025 \
        image:=/camera/image_raw \
        camera:=/camera

Then copy the K, D, R, P matrices from the resulting YAML into this file.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo


# ─── Approximate Logitech C270 calibration at 960x720 ────────────────────────
#   These values are reasonable defaults based on the C270's ~60° horizontal FOV
#   and are sufficient for visual loop-closure detection.
#   Replace with values from `ros2 run camera_calibration cameracalibrator`
#   for better feature matching accuracy.
#
#   Focal lengths estimated from FOV:
#     fx = (width/2) / tan(HFOV/2) = 480 / tan(30°) ≈ 831.0
#     fy ≈ fx (square pixels)
#   Principal point assumed at image centre:
#     cx = width/2  = 480.0
#     cy = height/2 = 360.0
# ─────────────────────────────────────────────────────────────────────────────
CAM_WIDTH  = 960
CAM_HEIGHT = 720
FX         = 831.0
FY         = 831.0
CX         = 480.0
CY         = 360.0
DISTORTION = [0.0, 0.0, 0.0, 0.0, 0.0]   # Replace with calibrated D


def _build_camera_info(frame_id: str) -> CameraInfo:
    """Build a static CameraInfo message template (timestamp filled per-frame)."""
    msg = CameraInfo()
    msg.header.frame_id = frame_id
    msg.width  = CAM_WIDTH
    msg.height = CAM_HEIGHT

    msg.distortion_model = 'plumb_bob'
    msg.d = DISTORTION

    # 3×3 intrinsic matrix (row-major)
    msg.k = [FX,  0.0, CX,
             0.0, FY,  CY,
             0.0, 0.0, 1.0]

    # Rectification matrix — identity for monocular
    msg.r = [1.0, 0.0, 0.0,
             0.0, 1.0, 0.0,
             0.0, 0.0, 1.0]

    # 3×4 projection matrix — same as K with zero 4th column (no stereo baseline)
    msg.p = [FX,  0.0, CX,  0.0,
             0.0, FY,  CY,  0.0,
             0.0, 0.0, 1.0, 0.0]

    return msg


class CameraInfoPublisher(Node):
    """
    Republishes CameraInfo whose header.stamp matches each incoming Image.

    This ensures that RTAB-Map's ApproximateTime message synchronizer can
    pair every image frame with its corresponding CameraInfo.
    """

    def __init__(self):
        super().__init__('camera_info_publisher')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('frame_id', 'camera_link')
        frame_id = self.get_parameter('frame_id').value

        self._template = _build_camera_info(frame_id)

        # ── QoS ───────────────────────────────────────────────────────────────
        # RELIABLE + KEEP_LAST to match how camera_publisher_node publishes /image_raw
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self._pub = self.create_publisher(CameraInfo, '/camera/camera_info', qos)

        self._sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self._image_cb,
            qos,
        )

        self.get_logger().info(
            f'camera_info_publisher ready — '
            f'frame_id={frame_id!r}  '
            f'resolution={CAM_WIDTH}x{CAM_HEIGHT}  '
            f'fx={FX}  fy={FY}  cx={CX}  cy={CY}'
        )

    def _image_cb(self, img_msg: Image):
        """Mirror the image timestamp onto a CameraInfo and publish."""
        msg = CameraInfo()
        msg.header.stamp    = img_msg.header.stamp      # ← timestamp must match exactly
        msg.header.frame_id = self._template.header.frame_id
        msg.width           = self._template.width
        msg.height          = self._template.height
        msg.distortion_model = self._template.distortion_model
        msg.d               = self._template.d
        msg.k               = self._template.k
        msg.r               = self._template.r
        msg.p               = self._template.p
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
