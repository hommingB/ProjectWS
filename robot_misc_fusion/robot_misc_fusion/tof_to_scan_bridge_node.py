#!/usr/bin/env python3
"""
ToF → Virtual LaserScan Bridge
--------------------------------
Converts the two VL53L0X Range messages into a sparse LaserScan that
nav2's costmap2d can consume via the range_sensor_layer or as a
supplementary scan topic.

This is useful when you want the ToF obstacle readings to appear on
the local costmap WITHOUT requiring a separate sensor layer plugin —
it just adds two "fake" laser beams at the known sensor angles.

Sensor geometry (from your diagram):
  Robot frame: x = forward, y = left
  Left  sensor: offset (+0.225, +0.18) m from base_link, bearing = +55°
  Right sensor: offset (+0.225, -0.18) m from base_link, bearing = -55°

  The bearing angle is measured from the sensor's own x-axis (forward).
  We inject two rays into a small LaserScan at those angles.

Published topic:
  /tof/scan  — sensor_msgs/LaserScan  (2-beam virtual scan)

Parameters:
  left_sensor_bearing_deg  : float = 55.0
  right_sensor_bearing_deg : float = -55.0
  scan_frame_id            : str   = 'base_link'
  range_max_m              : float = 1.2
  range_min_m              : float = 0.03
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range, LaserScan
import math


class TofToScanBridgeNode(Node):

    def __init__(self):
        super().__init__('tof_to_scan_bridge_node')

        self.declare_parameter('left_sensor_bearing_deg',   55.0)
        self.declare_parameter('right_sensor_bearing_deg', -55.0)
        self.declare_parameter('scan_frame_id',             'base_link')
        self.declare_parameter('range_max_m',               1.2)
        self.declare_parameter('range_min_m',               0.03)

        left_deg   = self.get_parameter('left_sensor_bearing_deg').value
        right_deg  = self.get_parameter('right_sensor_bearing_deg').value
        self.frame = self.get_parameter('scan_frame_id').value
        self.r_max = self.get_parameter('range_max_m').value
        self.r_min = self.get_parameter('range_min_m').value

        self.left_angle  = math.radians(left_deg)
        self.right_angle = math.radians(right_deg)

        self.d_left  = self.r_max
        self.d_right = self.r_max

        # Build the angle array: two beams only
        self.angles = sorted([self.right_angle, self.left_angle])
        self.angle_min = self.angles[0]
        self.angle_max = self.angles[-1]
        self.angle_inc = self.angle_max - self.angle_min  # gap between two beams

        self.create_subscription(Range, '/tof/left',  self._cb_left,  10)
        self.create_subscription(Range, '/tof/right', self._cb_right, 10)
        self.pub_scan = self.create_publisher(LaserScan, '/tof/scan', 10)

        self.get_logger().info(
            f'ToF→LaserScan bridge: beams at '
            f'{math.degrees(self.left_angle):.0f}° and '
            f'{math.degrees(self.right_angle):.0f}°')

    def _cb_left(self, msg: Range):
        self.d_left = msg.range
        self._publish()

    def _cb_right(self, msg: Range):
        self.d_right = msg.range
        self._publish()

    def _publish(self):
        scan = LaserScan()
        scan.header.stamp    = self.get_clock().now().to_msg()
        scan.header.frame_id = self.frame
        scan.angle_min       = self.angle_min
        scan.angle_max       = self.angle_max
        scan.angle_increment = self.angle_inc
        scan.time_increment  = 0.0
        scan.scan_time       = 0.05
        scan.range_min       = self.r_min
        scan.range_max       = self.r_max

        # Order must match angle order (right = smaller angle first)
        scan.ranges = [float(self.d_right), float(self.d_left)]
        self.pub_scan.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = TofToScanBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
