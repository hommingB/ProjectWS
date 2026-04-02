#!/usr/bin/env python3
"""
ToF Safety Node
---------------
Subscribes to /tof/left, /tof/right (and /tof/fused) and intercepts
the navigation cmd_vel, zeroing or scaling linear.x when an obstacle
is detected within the danger zone.

Topic graph:
  nav2 → /cmd_vel_nav  ──► [this node] ──► /cmd_vel  ──► wheel driver
                              ↑
                        /tof/left
                        /tof/right

Zones (configurable):
  STOP  zone  : distance < stop_dist_m   → zero velocity completely
  SLOW  zone  : distance < slow_dist_m   → scale velocity by (d/slow_dist)
  CLEAR zone  : distance >= slow_dist_m  → pass through unchanged

Parameters:
  stop_dist_m   : float = 0.20   (20 cm hard stop)
  slow_dist_m   : float = 0.45   (45 cm start slowing)
  input_topic   : str   = /cmd_vel_nav
  output_topic  : str   = /cmd_vel
  watchdog_s    : float = 0.5    (stop if no ToF data for this long)
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
import math


class TofSafetyNode(Node):

    def __init__(self):
        super().__init__('tof_safety_node')

        # ── Parameters ──────────────────────────────────────────────
        self.declare_parameter('stop_dist_m',   0.20)
        self.declare_parameter('slow_dist_m',   0.45)
        self.declare_parameter('input_topic',   '/cmd_vel_nav')
        self.declare_parameter('output_topic',  '/cmd_vel')
        self.declare_parameter('watchdog_s',    0.5)

        self.stop_dist   = self.get_parameter('stop_dist_m').value
        self.slow_dist   = self.get_parameter('slow_dist_m').value
        input_topic      = self.get_parameter('input_topic').value
        output_topic     = self.get_parameter('output_topic').value
        watchdog_s       = self.get_parameter('watchdog_s').value

        # ── State ────────────────────────────────────────────────────
        self.dist_left   = 1.2
        self.dist_right  = 1.2
        self.last_tof_t  = self.get_clock().now()

        # ── Pub / Sub ────────────────────────────────────────────────
        self.pub_cmd = self.create_publisher(Twist, output_topic, 10)

        self.create_subscription(Range, '/tof/left',
                                 self._cb_left,  10)
        self.create_subscription(Range, '/tof/right',
                                 self._cb_right, 10)
        self.create_subscription(Twist, input_topic,
                                 self._cb_cmd,   10)

        # Watchdog timer — publishes zero twist if ToF data goes stale
        self.create_timer(watchdog_s * 0.5, self._watchdog_cb)
        self._watchdog_dur = Duration(seconds=watchdog_s)

        self.get_logger().info(
            f'ToF safety node ready | '
            f'stop={self.stop_dist*100:.0f}cm '
            f'slow={self.slow_dist*100:.0f}cm')

    # ----------------------------------------------------------------
    def _cb_left(self, msg: Range):
        self.dist_left  = msg.range
        self.last_tof_t = self.get_clock().now()

    def _cb_right(self, msg: Range):
        self.dist_right = msg.range
        self.last_tof_t = self.get_clock().now()

    # ----------------------------------------------------------------
    def _obstacle_scale(self) -> float:
        """
        Returns a scale factor [0.0, 1.0] for forward velocity.
        Left / right sensors each contribute independently;
        worst case (minimum scale) wins.
        """
        d_min = min(self.dist_left, self.dist_right)

        if d_min <= self.stop_dist:
            return 0.0
        if d_min <= self.slow_dist:
            # Linear ramp: 0 at stop_dist → 1 at slow_dist
            return (d_min - self.stop_dist) / (self.slow_dist - self.stop_dist)
        return 1.0

    # ----------------------------------------------------------------
    def _cb_cmd(self, msg: Twist):
        """Intercept nav cmd_vel, apply obstacle safety, republish."""
        scale = self._obstacle_scale()
        safe  = Twist()

        # Scale forward motion; always allow rotation so robot can escape
        safe.linear.x  = msg.linear.x  * scale
        safe.linear.y  = msg.linear.y  * scale
        safe.linear.z  = msg.linear.z
        safe.angular.x = msg.angular.x
        safe.angular.y = msg.angular.y
        safe.angular.z = msg.angular.z  # rotation not blocked

        # Log state changes only (avoid spam)
        if scale == 0.0:
            self.get_logger().warn(
                f'OBSTACLE STOP — L:{self.dist_left*100:.1f}cm '
                f'R:{self.dist_right*100:.1f}cm',
                throttle_duration_sec=1.0)
        elif scale < 1.0:
            self.get_logger().info(
                f'Slowing: scale={scale:.2f} '
                f'L:{self.dist_left*100:.1f}cm '
                f'R:{self.dist_right*100:.1f}cm',
                throttle_duration_sec=1.0)

        self.pub_cmd.publish(safe)

    # ----------------------------------------------------------------
    def _watchdog_cb(self):
        """If ToF data is stale, publish zero velocity as safety stop."""
        age = self.get_clock().now() - self.last_tof_t
        if age > self._watchdog_dur:
            self.get_logger().warn(
                'ToF data stale — publishing zero velocity!',
                throttle_duration_sec=2.0)
            self.pub_cmd.publish(Twist())  # zero twist


# ════════════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = TofSafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
