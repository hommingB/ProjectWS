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
    Republishes both Image and CameraInfo onto synchronized topics with matching
    timestamps and a unified TF frame_id (e.g. camera_link), dynamically adjusting
    calibration parameters based on the incoming image resolution.
    """

    def __init__(self):
        super().__init__('camera_info_publisher')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('frame_id', 'camera_link')
        self.frame_id = self.get_parameter('frame_id').value

        # ── QoS ───────────────────────────────────────────────────────────────
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # We will publish synchronized versions of both image and info
        self._img_pub = self.create_publisher(Image, '/camera/image_raw_sync', qos)
        self._info_pub = self.create_publisher(CameraInfo, '/camera/camera_info_sync', qos)

        self._sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self._image_cb,
            qos,
        )

        self.get_logger().info(
            f'camera_info_publisher ready — mapping /camera/image_raw to frame_id={self.frame_id!r}'
        )

    def _image_cb(self, img_msg: Image):
        # 1. Republish the image with corrected frame_id
        synced_img = img_msg
        synced_img.header.frame_id = self.frame_id
        self._img_pub.publish(synced_img)

        # 2. Build and publish the matching camera info dynamically using the incoming resolution
        w = img_msg.width
        h = img_msg.height
        
        # Scale focal lengths and principal point based on incoming resolution
        # Logitech C270 standard: fx ≈ w * 0.865
        fx = float(w) * 0.865
        fy = fx
        cx = float(w) / 2.0
        cy = float(h) / 2.0

        msg = CameraInfo()
        msg.header.stamp = img_msg.header.stamp
        msg.header.frame_id = self.frame_id
        msg.width = w
        msg.height = h

        msg.distortion_model = 'plumb_bob'
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        msg.k = [fx,  0.0, cx,
                 0.0, fy,  cy,
                 0.0, 0.0, 1.0]

        msg.r = [1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0]

        msg.p = [fx,  0.0, cx,  0.0,
                 0.0, fy,  cy,  0.0,
                 0.0, 0.0, 1.0, 0.0]

        self._info_pub.publish(msg)


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
