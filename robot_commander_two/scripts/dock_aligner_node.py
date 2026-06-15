#!/usr/bin/env python3
"""Final dock alignment: slow reverse when mode_manager enters RESTING.

Nav2 brings the robot to the dock pose; this node creeps backward the last
few centimetres (no contact sensor). For demos, it can publish /charging_status
so mode_manager transitions RESTING → CHARGING.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String


class DockAlignerNode(Node):
    IDLE = 'idle'
    ACTIVE = 'active'
    DONE = 'done'

    def __init__(self) -> None:
        super().__init__('dock_aligner')

        self.declare_parameter('reverse_speed', 0.05)          # m/s, positive magnitude
        self.declare_parameter('reverse_duration_sec', 3.0)
        self.declare_parameter('reverse_distance_m', 0.15)
        self.declare_parameter('cmd_rate_hz', 10.0)
        self.declare_parameter('cmd_vel_topic', 'docking/cmd_vel')
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('simulate_charging_contact', True)
        self.declare_parameter('stop_hold_sec', 0.3)           # keep publishing zero after stop

        self._state = self.IDLE
        self._prev_robot_state = ''
        self._align_done = False
        self._elapsed = 0.0
        self._stop_hold = 0.0
        self._start_pose = None
        self._start_yaw = 0.0
        self._latest_odom = None

        cmd_topic = self.get_parameter('cmd_vel_topic').value
        self._pub_cmd = self.create_publisher(TwistStamped, cmd_topic, 10)
        self._pub_charging = self.create_publisher(Bool, 'charging_status', 10)

        self.create_subscription(String, 'robot_state', self._on_robot_state, 10)
        odom_topic = self.get_parameter('odom_topic').value
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)

        rate = self.get_parameter('cmd_rate_hz').value
        self.create_timer(1.0 / rate, self._on_timer)

        speed = self.get_parameter('reverse_speed').value
        duration = self.get_parameter('reverse_duration_sec').value
        distance = self.get_parameter('reverse_distance_m').value
        self.get_logger().info(
            f'Dock aligner ready – reverse {speed:.2f} m/s for up to {duration:.1f} s / {distance:.2f} m on RESTING'
        )

    def _on_odom(self, msg: Odometry) -> None:
        self._latest_odom = msg

    def _on_robot_state(self, msg: String) -> None:
        state = msg.data.strip()

        if state == 'RESTING' and self._prev_robot_state != 'RESTING':
            if not self._align_done:
                self._begin_align()
        elif state not in ('RESTING', 'CHARGING'):
            self._align_done = False
            if self._state != self.IDLE:
                self.get_logger().info(f'Left dock cycle ({state}) – reset')
                self._stop_motion()
                self._state = self.IDLE

        self._prev_robot_state = state

    def _begin_align(self) -> None:
        self._state = self.ACTIVE
        self._elapsed = 0.0
        self._stop_hold = 0.0
        self._capture_start_pose()
        self.get_logger().info('RESTING detected – starting slow reverse')

    def _capture_start_pose(self) -> None:
        if self._latest_odom is None:
            self._start_pose = None
            self._start_yaw = 0.0
            return
        p = self._latest_odom.pose.pose.position
        self._start_pose = (p.x, p.y)
        q = self._latest_odom.pose.pose.orientation
        self._start_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def _distance_reversed(self) -> float:
        if self._start_pose is None or self._latest_odom is None:
            return 0.0
        p = self._latest_odom.pose.pose.position
        dx = p.x - self._start_pose[0]
        dy = p.y - self._start_pose[1]
        hx = math.cos(self._start_yaw)
        hy = math.sin(self._start_yaw)
        forward = dx * hx + dy * hy
        return max(0.0, -forward)

    def _on_timer(self) -> None:
        dt = 1.0 / self.get_parameter('cmd_rate_hz').value

        if self._state == self.ACTIVE:
            self._elapsed += dt
            speed = self.get_parameter('reverse_speed').value
            max_t = self.get_parameter('reverse_duration_sec').value
            max_d = self.get_parameter('reverse_distance_m').value
            dist = self._distance_reversed()

            if self._elapsed >= max_t or dist >= max_d:
                self.get_logger().info(
                    f'Reverse complete (t={self._elapsed:.2f} s, d={dist:.3f} m) – stopping'
                )
                self._finish_align()
                return

            twist = TwistStamped()
            twist.header.stamp = self.get_clock().now().to_msg()
            twist.header.frame_id = 'base_link'
            twist.twist.linear.x = -abs(speed)
            self._pub_cmd.publish(twist)
            return

        if self._state == self.DONE:
            self._stop_hold += dt
            self._publish_zero_cmd()
            hold = self.get_parameter('stop_hold_sec').value
            if self._stop_hold >= hold:
                self._state = self.IDLE
            return

    def _finish_align(self) -> None:
        self._align_done = True
        self._state = self.DONE
        self._stop_hold = 0.0
        self._publish_zero_cmd()

        if self.get_parameter('simulate_charging_contact').value:
            msg = Bool()
            msg.data = True
            self._pub_charging.publish(msg)
            self.get_logger().info(
                'Published charging_status=true (demo – disable simulate_charging_contact for real hardware)',
            )

    def _stop_motion(self) -> None:
        self._publish_zero_cmd()

    def _publish_zero_cmd(self) -> None:
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = 'base_link'
        self._pub_cmd.publish(twist)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DockAlignerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_motion()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
