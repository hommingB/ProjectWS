#!/usr/bin/env python3
"""
person_detection_fusion_node.py  — TF2-corrected camera/lidar fusion
---------------------------------------------------------------------
Key change from the naive version:
  Instead of computing a bearing in camera frame and naively indexing
  the lidar scan (which assumes both sensors are co-located), we now:

  1. Unproject each valid lidar range into a 3-D point in laser frame.
  2. Transform that point into camera_optical frame using TF2.
  3. Compute the bearing of the transformed point in camera frame.
  4. Match against the bounding-box centre bearing.

This correctly handles any sensor offset (translation + rotation) as
long as the static TF tree is published.

Required TF frames — add to your URDF or launch file:
  base_link → camera_optical
  base_link → laser

Example (launch file static_transform_publisher):
  Node(package='tf2_ros', executable='static_transform_publisher',
       arguments=['0.05','0','0.15','0','0','0','base_link','camera_optical'])
  Node(package='tf2_ros', executable='static_transform_publisher',
       arguments=['-0.03','0','0.10','0','0','0','base_link','laser'])

Adjust x/y/z to your actual mounting.
"""

import rclpy
from rclpy.node import Node
import math

from sensor_msgs.msg import Image, CameraInfo, LaserScan
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import Float32
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D
from cv_bridge import CvBridge
import cv2

import tf2_ros
import tf2_geometry_msgs

try:
    import onnxruntime as ort
    ONNX_OK = True
except ImportError:
    ONNX_OK = False


class PersonDetectionFusionNode(Node):

    def __init__(self):
        super().__init__('person_detection_fusion_node')

        self.declare_parameter('use_hog',            True)
        self.declare_parameter('onnx_model_path',    '')
        self.declare_parameter('confidence_thresh',   0.5)
        self.declare_parameter('img_width',           640)
        self.declare_parameter('img_height',          480)
        self.declare_parameter('publish_debug_img',   True)
        self.declare_parameter('camera_hfov_deg',     60.0)
        self.declare_parameter('approach_dist_m',     0.80)
        self.declare_parameter('person_goal_frame',   'map')
        self.declare_parameter('camera_frame',        'camera_optical')
        self.declare_parameter('laser_frame',         'laser')
        self.declare_parameter('tf_timeout_s',        0.1)

        self.use_hog       = self.get_parameter('use_hog').value
        self.onnx_path     = self.get_parameter('onnx_model_path').value
        self.conf_thresh   = self.get_parameter('confidence_thresh').value
        self.img_w         = self.get_parameter('img_width').value
        self.img_h         = self.get_parameter('img_height').value
        self.debug_img     = self.get_parameter('publish_debug_img').value
        self.hfov          = math.radians(
                             self.get_parameter('camera_hfov_deg').value)
        self.approach_dist = self.get_parameter('approach_dist_m').value
        self.goal_frame    = self.get_parameter('person_goal_frame').value
        self.cam_frame     = self.get_parameter('camera_frame').value
        self.laser_frame   = self.get_parameter('laser_frame').value
        self.tf_timeout    = rclpy.duration.Duration(
                             seconds=self.get_parameter('tf_timeout_s').value)

        # TF2
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Detector
        if self.use_hog:
            self.hog = cv2.HOGDescriptor()
            self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
            self.get_logger().info('Using OpenCV HOG person detector')
        elif ONNX_OK and self.onnx_path:
            self.session = ort.InferenceSession(self.onnx_path)
            self.get_logger().info(f'Using ONNX model: {self.onnx_path}')
        else:
            self.get_logger().warn('No detector available; set use_hog=true')

        self.bridge      = CvBridge()
        self.camera_fx   = None
        self.camera_cx   = None
        self.latest_scan: LaserScan = None

        self.create_subscription(Image,      '/camera/image_raw',
                                 self._cb_image,   1)
        self.create_subscription(CameraInfo, '/camera/camera_info',
                                 self._cb_caminfo, 1)
        self.create_subscription(LaserScan,  '/scan',
                                 self._cb_scan,    5)

        self.pub_detect  = self.create_publisher(
            Detection2DArray, '/person_detection/detections', 10)
        self.pub_bearing = self.create_publisher(
            Float32, '/person_detection/target_bearing', 10)
        self.pub_goal    = self.create_publisher(
            PoseStamped, '/person_detection/target_goal', 10)
        if self.debug_img:
            self.pub_debug = self.create_publisher(
                Image, '/person_detection/image_annotated', 5)

        self.get_logger().info(
            f'PersonDetectionFusion ready  '
            f'cam={self.cam_frame}  laser={self.laser_frame}')

    def _cb_caminfo(self, msg: CameraInfo):
        if self.camera_fx is None:
            self.camera_fx = msg.k[0]
            self.camera_cx = msg.k[2]

    def _cb_scan(self, msg: LaserScan):
        self.latest_scan = msg

    # ── TF2-corrected bearing map ─────────────────────────────────────
    def _build_camera_bearing_map(self, stamp) -> list:
        """
        Transform every valid lidar hit into camera frame via TF2,
        then compute its bearing in camera frame.
        Returns [(bearing_rad, range_m), ...] sorted by bearing.

        This is the correct approach for offset sensors.
        Naive approach (just remapping lidar angle to camera FOV) is
        WRONG whenever the sensors are not co-located.
        """
        scan = self.latest_scan
        if scan is None:
            return []

        try:
            tf = self.tf_buffer.lookup_transform(
                self.cam_frame,
                self.laser_frame,
                stamp,
                self.tf_timeout)
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(
                f'TF {self.laser_frame}→{self.cam_frame}: {e}',
                throttle_duration_sec=2.0)
            return []

        result = []
        for i, r in enumerate(scan.ranges):
            if not (scan.range_min < r < scan.range_max):
                continue

            angle = scan.angle_min + i * scan.angle_increment
            pt = PointStamped()
            pt.header.frame_id = self.laser_frame
            pt.header.stamp    = stamp
            pt.point.x = r * math.cos(angle)
            pt.point.y = r * math.sin(angle)
            pt.point.z = 0.0

            try:
                pt_cam = tf2_geometry_msgs.do_transform_point(pt, tf)
            except Exception:
                continue

            # camera_optical: z=forward, x=right, y=down
            if pt_cam.point.z <= 0:
                continue

            bearing = math.atan2(pt_cam.point.x, pt_cam.point.z)
            result.append((bearing, r))

        result.sort(key=lambda p: p[0])
        return result

    def _closest_lidar_range(self, target_bearing: float,
                              bearing_map: list,
                              window_rad: float = 0.10) -> float | None:
        candidates = [r for b, r in bearing_map
                      if abs(b - target_bearing) <= window_rad]
        return min(candidates) if candidates else None

    def _bearing_from_pixel(self, cx_px: float) -> float:
        if self.camera_fx:
            return math.atan2(cx_px - self.camera_cx, self.camera_fx)
        return ((cx_px / self.img_w) - 0.5) * self.hfov

    def _detect_hog(self, frame):
        small = cv2.resize(frame, (self.img_w // 2, self.img_h // 2))
        rects, _ = self.hog.detectMultiScale(
            small, winStride=(8, 8), padding=(4, 4), scale=1.05)
        return [(x*2, y*2, w*2, h*2) for (x, y, w, h) in rects]

    def _detect_onnx(self, frame):
        blob = cv2.dnn.blobFromImage(frame, 1/255.0, (300, 300), swapRB=True)
        inputs  = {self.session.get_inputs()[0].name: blob}
        outputs = self.session.run(None, inputs)
        boxes = []
        for det in outputs[0][0]:
            if int(det[1]) == 1 and det[2] >= self.conf_thresh:
                x1, y1 = int(det[3]*frame.shape[1]), int(det[4]*frame.shape[0])
                x2, y2 = int(det[5]*frame.shape[1]), int(det[6]*frame.shape[0])
                boxes.append((x1, y1, x2-x1, y2-y1))
        return boxes

    def _cb_image(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        boxes = (self._detect_hog(frame) if self.use_hog
                 else (self._detect_onnx(frame)
                       if ONNX_OK and hasattr(self, 'session') else []))

        # Build the TF2-corrected bearing map once per frame
        bearing_map = self._build_camera_bearing_map(msg.header.stamp)
        tf_ok = len(bearing_map) > 0

        det_arr = Detection2DArray()
        det_arr.header.stamp    = msg.header.stamp
        det_arr.header.frame_id = self.cam_frame

        best_bearing = None
        best_dist    = float('inf')

        for (x, y, w, h) in boxes:
            cx = x + w / 2.0
            det = Detection2D()
            det.header = det_arr.header
            bb = BoundingBox2D()
            bb.center.position.x = float(cx)
            bb.center.position.y = float(y + h / 2.0)
            bb.size_x, bb.size_y = float(w), float(h)
            det.bbox = bb
            det_arr.detections.append(det)

            bearing = self._bearing_from_pixel(cx)
            dist = (self._closest_lidar_range(bearing, bearing_map)
                    if tf_ok else None)
            if dist is None:
                dist = (0.45 * self.camera_fx / w) if self.camera_fx else 1.0

            if dist < best_dist:
                best_dist, best_bearing = dist, bearing

            if self.debug_img:
                color = (0, 255, 0) if dist > self.approach_dist else (0, 0, 255)
                cv2.rectangle(frame, (x, y), (x+w, y+h), color, 2)
                cv2.putText(frame,
                            f'{dist:.2f}m' + ('' if tf_ok else ' [no TF]'),
                            (x, y-8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        self.pub_detect.publish(det_arr)

        if best_bearing is not None and best_dist < 4.0:
            b_msg = Float32()
            b_msg.data = float(best_bearing)
            self.pub_bearing.publish(b_msg)

            if best_dist > self.approach_dist:
                goal = PoseStamped()
                goal.header.stamp    = msg.header.stamp
                goal.header.frame_id = 'base_link'
                goal_dist = best_dist - self.approach_dist
                goal.pose.position.x = goal_dist * math.cos(best_bearing)
                goal.pose.position.y = goal_dist * math.sin(best_bearing)
                goal.pose.orientation.w = 1.0
                self.pub_goal.publish(goal)

        if self.debug_img:
            self.pub_debug.publish(
                self.bridge.cv2_to_imgmsg(frame, encoding='bgr8'))


def main(args=None):
    rclpy.init(args=args)
    node = PersonDetectionFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()