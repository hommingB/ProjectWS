#!/usr/bin/env python3
"""
Nav2 Performance Monitor Node.
Watches for every Nav2 service request's command_id and calculates the distance and yaw angle errors
from the requested destination point using TF lookup.
Measures the moving time of each request (accounting for pauses) and records logs to screen and a CSV file.
"""

import os
import csv
import json
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from robot_commander_two.msg import ServiceRequest
from geometry_msgs.msg import PoseStamped
import tf2_ros
from rclpy.duration import Duration


class Nav2PerformanceMonitor(Node):
    def __init__(self) -> None:
        super().__init__('nav2_performance_monitor')

        # Parameters
        self.declare_parameter('log_file_path', '~/ros2_project/src/robot_commander_two/nav2_performance_log.csv')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('periodic_log_rate', 1.0) # Hz

        self.log_file = self.get_parameter('log_file_path').value
        self.target_frame = self.get_parameter('target_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.periodic_log_rate = self.get_parameter('periodic_log_rate').value

        # TF2 Listener setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # State tracking dictionary: command_id -> request_info
        self.requests = {}
        self.latest_goal = None

        # Ensure log directory and CSV headers exist
        self._init_csv()

        # Subscriptions
        self.create_subscription(ServiceRequest, 'service_request', self._service_request_cb, 10)
        self.create_subscription(PoseStamped, 'goal_pose', self._goal_pose_cb, 10)
        self.create_subscription(String, 'service_feedback', self._service_feedback_cb, 10)

        # Periodic timer for logging errors while executing
        self.create_timer(1.0 / self.periodic_log_rate, self._periodic_log_cb)

        self.get_logger().info(
            f"Nav2 Performance Monitor active.\n"
            f"  Target Frame: {self.target_frame}\n"
            f"  Base Frame: {self.base_frame}\n"
            f"  Logging CSV: {self.log_file}"
        )

    def _init_csv(self) -> None:
        try:
            log_dir = os.path.dirname(self.log_file)
            if log_dir:
                os.makedirs(log_dir, exist_ok=True)
            
            # Check if file exists, write header if not
            file_exists = os.path.exists(self.log_file)
            with open(self.log_file, 'a', newline='') as f:
                writer = csv.writer(f)
                if not file_exists or os.stat(self.log_file).st_size == 0:
                    writer.writerow([
                        'command_id', 'status',
                        'target_x_m', 'target_y_m', 'target_yaw_deg',
                        'start_x_m', 'start_y_m', 'start_yaw_deg',
                        'end_x_m', 'end_y_m', 'end_yaw_deg',
                        'initial_dist_err_cm', 'initial_yaw_err_deg',
                        'final_dist_err_cm', 'final_yaw_err_deg',
                        'moving_duration_sec'
                    ])
            self.get_logger().info(f"Initialized CSV log file at {self.log_file}")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize CSV log file: {e}")

    def normalize_angle(self, angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def m_to_cm(value: float) -> float:
        """Convert meters to centimeters."""
        return value * 100.0

    @staticmethod
    def rad_to_deg(value: float) -> float:
        """Convert radians to degrees, preserving sign."""
        return math.degrees(value)

    def _get_robot_pose(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.base_frame,
                now,
                timeout=Duration(seconds=0.1)
            )
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            q = trans.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )
            return x, y, yaw
        except Exception as e:
            # Throttle warnings to not flood the logs
            self.get_logger().warning(
                f"TF pose lookup failed ('{self.target_frame}' -> '{self.base_frame}'): {e}",
                throttle_duration_sec=5.0
            )
            return None

    def _service_request_cb(self, msg: ServiceRequest) -> None:
        command_id = msg.command_id
        if not command_id:
            return

        x_target = msg.destination.pose.position.x
        y_target = msg.destination.pose.position.y
        q = msg.destination.pose.orientation
        yaw_target = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        self.get_logger().info(f"Tracking ServiceRequest: '{command_id}' target=({x_target:.2f}, {y_target:.2f}, {yaw_target:.2f} rad)")

        self.requests[command_id] = {
            'x_target': x_target,
            'y_target': y_target,
            'yaw_target': yaw_target,
            'state': 'PENDING',
            'start_time': None,
            'last_resume_time': None,
            'accumulated_moving_time': 0.0, # seconds
            'start_pose': None,
            'initial_dist_err': None,
            'initial_yaw_err': None,
            'last_logged_errors': (None, None)
        }

    def _goal_pose_cb(self, msg: PoseStamped) -> None:
        # Cache the latest goal published to correlate with incoming executing tasks
        self.latest_goal = msg

    def _service_feedback_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            command_id = data.get('command_id')
            status = data.get('status')
        except Exception as e:
            self.get_logger().error(f"Malformed feedback JSON: {msg.data} - {e}")
            return

        if not command_id or not status:
            return

        # Ensure we have target coordinates. If not in self.requests, look up from cached goal_pose
        if command_id not in self.requests:
            if self.latest_goal is not None:
                x_target = self.latest_goal.pose.position.x
                y_target = self.latest_goal.pose.position.y
                q = self.latest_goal.pose.orientation
                yaw_target = math.atan2(
                    2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                )
                self.requests[command_id] = {
                    'x_target': x_target,
                    'y_target': y_target,
                    'yaw_target': yaw_target,
                    'state': 'PENDING',
                    'start_time': None,
                    'last_resume_time': None,
                    'accumulated_moving_time': 0.0,
                    'start_pose': None,
                    'initial_dist_err': None,
                    'initial_yaw_err': None,
                    'last_logged_errors': (None, None)
                }
                self.get_logger().info(
                    f"Correlated internal task '{command_id}' with latest goal: "
                    f"({x_target:.2f}, {y_target:.2f}, {yaw_target:.2f} rad)"
                )
                self.latest_goal = None # consume it
            else:
                # Fallback to (0,0,0) targets if completely untracked (e.g. node started mid-execution)
                self.requests[command_id] = {
                    'x_target': 0.0,
                    'y_target': 0.0,
                    'yaw_target': 0.0,
                    'state': 'PENDING',
                    'start_time': None,
                    'last_resume_time': None,
                    'accumulated_moving_time': 0.0,
                    'start_pose': None,
                    'initial_dist_err': None,
                    'initial_yaw_err': None,
                    'last_logged_errors': (None, None)
                }
                self.get_logger().warning(
                    f"Received feedback for untracked task '{command_id}'. Target set to (0,0,0)."
                )

        req = self.requests[command_id]
        now = self.get_clock().now()

        # State transition handling
        if status == 'EXECUTING':
            if req['state'] != 'EXECUTING':
                # Transition to executing (start moving or resume from pause)
                req['state'] = 'EXECUTING'
                req['last_resume_time'] = now

                # Capture start/initial metrics if executing for the first time
                if req['start_time'] is None:
                    req['start_time'] = now
                    pose = self._get_robot_pose()
                    if pose:
                        x, y, yaw = pose
                        req['start_pose'] = (x, y, yaw)
                        dist_err = math.sqrt((req['x_target'] - x)**2 + (req['y_target'] - y)**2)
                        yaw_err = self.normalize_angle(req['yaw_target'] - yaw)
                        req['initial_dist_err'] = dist_err
                        req['initial_yaw_err'] = yaw_err
                        self.get_logger().info(
                            f"Task '{command_id}' STARTING execution.\n"
                            f"  Start Pose: ({x:.4f} m, {y:.4f} m, {math.degrees(yaw):.2f} deg)\n"
                            f"  Initial Errors -> Dist: {self.m_to_cm(dist_err):.2f} cm, "
                            f"Yaw: {self.rad_to_deg(yaw_err):+.2f} deg"
                        )
                    else:
                        self.get_logger().warning(f"Task '{command_id}' started, but robot pose is unavailable.")

        elif status == 'PAUSED':
            if req['state'] == 'EXECUTING':
                # Accumulate moving time
                duration = (now - req['last_resume_time']).nanoseconds / 1e9
                req['accumulated_moving_time'] += duration
                req['state'] = 'PAUSED'
                self.get_logger().info(f"Task '{command_id}' PAUSED. Accumulated moving time: {req['accumulated_moving_time']:.2f} s")

        elif status in ('SUCCEEDED', 'FAILED', 'CANCELED', 'PREEMPTED', 'PREEMPTED_BY_DOCK', 'REJECTED'):
            # Terminal states: task finished
            if req['state'] == 'EXECUTING':
                duration = (now - req['last_resume_time']).nanoseconds / 1e9
                req['accumulated_moving_time'] += duration
            
            req['state'] = 'TERMINATED'
            
            # Fetch final pose and calculate errors
            end_pose = self._get_robot_pose()
            final_dist_err = None
            final_yaw_err = None
            x_end, y_end, yaw_end = (None, None, None)

            if end_pose:
                x_end, y_end, yaw_end = end_pose
                final_dist_err = math.sqrt((req['x_target'] - x_end)**2 + (req['y_target'] - y_end)**2)
                final_yaw_err = self.normalize_angle(req['yaw_target'] - yaw_end)

            start_pose = req['start_pose'] or (0.0, 0.0, 0.0)
            initial_dist_err = req['initial_dist_err']
            initial_yaw_err = req['initial_yaw_err']
            moving_time = req['accumulated_moving_time']

            # Print terminal details
            log_msg = (
                f"Task '{command_id}' TERMINATED with status: '{status}'\n"
                f"  Total Moving Time: {moving_time:.2f} s\n"
            )
            if end_pose:
                log_msg += (
                    f"  Final Pose: ({x_end:.4f} m, {y_end:.4f} m, {math.degrees(yaw_end):.2f} deg)\n"
                    f"  Final Errors -> Dist: {self.m_to_cm(final_dist_err):.2f} cm, "
                    f"Yaw: {self.rad_to_deg(final_yaw_err):+.2f} deg"
                )
            else:
                log_msg += "  Final Pose / Errors: UNKNOWN (TF lookup failed)"
            self.get_logger().info(log_msg)

            # Record to CSV
            self._write_to_csv(
                command_id, status,
                req['x_target'], req['y_target'], req['yaw_target'],
                start_pose[0], start_pose[1], start_pose[2],
                x_end, y_end, yaw_end,
                initial_dist_err, initial_yaw_err,
                final_dist_err, final_yaw_err,
                moving_time
            )

    def _periodic_log_cb(self) -> None:
        # Periodically log the error for all actively executing tasks
        for command_id, req in list(self.requests.items()):
            if req['state'] == 'EXECUTING':
                pose = self._get_robot_pose()
                if pose:
                    x, y, yaw = pose
                    dist_err = math.sqrt((req['x_target'] - x)**2 + (req['y_target'] - y)**2)
                    yaw_err = self.normalize_angle(req['yaw_target'] - yaw)
                    
                    # Avoid logging duplicates if the error is exactly identical to previous log
                    if req['last_logged_errors'] != (dist_err, yaw_err):
                        req['last_logged_errors'] = (dist_err, yaw_err)
                        self.get_logger().info(
                            f"Task '{command_id}' EXECUTING: "
                            f"Dist Err={self.m_to_cm(dist_err):.2f} cm, "
                            f"Yaw Err={self.rad_to_deg(yaw_err):+.2f} deg"
                        )

    def _write_to_csv(self, command_id, status, tx, ty, tyaw, sx, sy, syaw, ex, ey, eyaw, init_d, init_y, fin_d, fin_y, duration) -> None:
        try:
            with open(self.log_file, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    command_id, status,
                    # Poses: x/y stay in metres, yaw converted to degrees
                    f"{tx:.4f}", f"{ty:.4f}", f"{math.degrees(tyaw):.2f}",
                    f"{sx:.4f}" if sx is not None else "N/A",
                    f"{sy:.4f}" if sy is not None else "N/A",
                    f"{math.degrees(syaw):.2f}" if syaw is not None else "N/A",
                    f"{ex:.4f}" if ex is not None else "N/A",
                    f"{ey:.4f}" if ey is not None else "N/A",
                    f"{math.degrees(eyaw):.2f}" if eyaw is not None else "N/A",
                    # Errors: distance in cm, yaw in degrees (signed)
                    f"{self.m_to_cm(init_d):.2f}" if init_d is not None else "N/A",
                    f"{self.rad_to_deg(init_y):+.2f}" if init_y is not None else "N/A",
                    f"{self.m_to_cm(fin_d):.2f}" if fin_d is not None else "N/A",
                    f"{self.rad_to_deg(fin_y):+.2f}" if fin_y is not None else "N/A",
                    f"{duration:.2f}"
                ])
        except Exception as e:
            self.get_logger().error(f"Failed to write log to CSV: {e}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Nav2PerformanceMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
