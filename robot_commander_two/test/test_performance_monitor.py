import os
import sys
import json
import math
import tempfile
import csv
import pytest
from unittest.mock import MagicMock, patch

import rclpy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, TransformStamped
from robot_commander_two.msg import ServiceRequest

# Add scripts directory to sys.path to import the node
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'scripts'))
from nav2_performance_monitor import Nav2PerformanceMonitor


@pytest.fixture(scope="function")
def rclpy_init_shutdown():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_performance_monitor_logic(rclpy_init_shutdown):
    # Create a temporary file for logging
    temp_dir = tempfile.gettempdir()
    temp_csv_path = os.path.join(temp_dir, 'test_nav2_performance_log.csv')
    if os.path.exists(temp_csv_path):
        os.remove(temp_csv_path)

    # Patch TF listener and buffer to prevent actual TF calls
    with patch('tf2_ros.TransformListener'), \
         patch('tf2_ros.Buffer') as mock_buffer_cls:
        
        # Setup mock buffer lookup_transform
        mock_buffer = mock_buffer_cls.return_value
        
        # We will dynamically change what lookup_transform returns
        mock_transform = MagicMock(spec=TransformStamped)
        mock_transform.transform.translation.x = 1.0
        mock_transform.transform.translation.y = 1.0
        mock_transform.transform.rotation.x = 0.0
        mock_transform.transform.rotation.y = 0.0
        # yaw = 0.0 -> q.z = 0.0, q.w = 1.0
        mock_transform.transform.rotation.z = 0.0
        mock_transform.transform.rotation.w = 1.0
        
        mock_buffer.lookup_transform.return_value = mock_transform

        # Initialize the node
        node = Nav2PerformanceMonitor()
        
        # Override parameters for testing
        node.log_file = temp_csv_path
        node._init_csv() # re-init with temp file
        
        assert os.path.exists(temp_csv_path)

        # 1. Test ServiceRequest Callback
        req_msg = ServiceRequest()
        req_msg.command_id = "TEST_REQ_01"
        req_msg.destination.pose.position.x = 3.0
        req_msg.destination.pose.position.y = 1.0
        # Target yaw = pi/2 (90 deg) -> q = (0, 0, sin(pi/4), cos(pi/4)) = (0, 0, 0.7071, 0.7071)
        req_msg.destination.pose.orientation.x = 0.0
        req_msg.destination.pose.orientation.y = 0.0
        req_msg.destination.pose.orientation.z = math.sin(math.pi / 4.0)
        req_msg.destination.pose.orientation.w = math.cos(math.pi / 4.0)

        node._service_request_cb(req_msg)
        
        assert "TEST_REQ_01" in node.requests
        req_data = node.requests["TEST_REQ_01"]
        assert req_data['x_target'] == 3.0
        assert req_data['y_target'] == 1.0
        assert math.isclose(req_data['yaw_target'], math.pi / 2.0, rel_tol=1e-4)
        assert req_data['state'] == 'PENDING'

        # 2. Test Feedback EXECUTING (Start)
        # Mock robot starts at (1.0, 1.0, 0.0) -> dist error = 2.0, yaw error = pi/2
        feedback_exec = String()
        feedback_exec.data = json.dumps({"command_id": "TEST_REQ_01", "status": "EXECUTING"})
        
        node._service_feedback_cb(feedback_exec)
        
        assert req_data['state'] == 'EXECUTING'
        assert req_data['start_pose'] == (1.0, 1.0, 0.0)
        assert math.isclose(req_data['initial_dist_err'], 2.0)
        assert math.isclose(req_data['initial_yaw_err'], math.pi / 2.0, rel_tol=1e-4)
        assert req_data['start_time'] is not None

        # 3. Simulate moving to a new pose and pausing
        # Let's change the mock pose to (2.0, 1.0, pi/4)
        mock_transform.transform.translation.x = 2.0
        mock_transform.transform.translation.y = 1.0
        mock_transform.transform.rotation.z = math.sin(math.pi / 8.0)
        mock_transform.transform.rotation.w = math.cos(math.pi / 8.0)
        
        # Mock time progression by manually adding a duration
        start_time = node.get_clock().now()
        req_data['last_resume_time'] = start_time
        
        feedback_pause = String()
        feedback_pause.data = json.dumps({"command_id": "TEST_REQ_01", "status": "PAUSED"})
        
        # We patch self.get_clock().now() to return a time 2.5 seconds later
        later_time = start_time + rclpy.duration.Duration(seconds=2.5)
        with patch.object(node.get_clock(), 'now', return_value=later_time):
            node._service_feedback_cb(feedback_pause)

        assert req_data['state'] == 'PAUSED'
        assert math.isclose(req_data['accumulated_moving_time'], 2.5)

        # 4. Resume executing
        feedback_resume = String()
        feedback_resume.data = json.dumps({"command_id": "TEST_REQ_01", "status": "EXECUTING"})
        
        resume_time = later_time + rclpy.duration.Duration(seconds=1.0) # 1 second pause
        with patch.object(node.get_clock(), 'now', return_value=resume_time):
            node._service_feedback_cb(feedback_resume)
            
        assert req_data['state'] == 'EXECUTING'
        assert req_data['last_resume_time'] == resume_time

        # 5. Succeed task
        # Let's change mock pose to final destination (3.0, 1.0, pi/2)
        mock_transform.transform.translation.x = 3.0
        mock_transform.transform.translation.y = 1.0
        mock_transform.transform.rotation.z = math.sin(math.pi / 4.0)
        mock_transform.transform.rotation.w = math.cos(math.pi / 4.0)
        
        feedback_success = String()
        feedback_success.data = json.dumps({"command_id": "TEST_REQ_01", "status": "SUCCEEDED"})
        
        success_time = resume_time + rclpy.duration.Duration(seconds=1.5)
        with patch.object(node.get_clock(), 'now', return_value=success_time):
            node._service_feedback_cb(feedback_success)

        assert req_data['state'] == 'TERMINATED'
        # Accumulated time: 2.5s (first execution) + 1.5s (second execution) = 4.0s
        assert math.isclose(req_data['accumulated_moving_time'], 4.0)

        # Check CSV file contents
        with open(temp_csv_path, 'r') as f:
            reader = csv.reader(f)
            rows = list(reader)

            assert len(rows) == 2  # Header + 1 record
            record = rows[1]

            # command_id, status
            assert record[0] == "TEST_REQ_01"
            assert record[1] == "SUCCEEDED"

            # targets: tx, ty in metres; target_yaw now stored in degrees
            assert float(record[2]) == 3.0
            assert float(record[3]) == 1.0
            assert math.isclose(float(record[4]), 90.0, abs_tol=0.01)   # pi/2 rad → 90.00 deg

            # starts: sx, sy in metres; start_yaw in degrees
            assert float(record[5]) == 1.0
            assert float(record[6]) == 1.0
            assert math.isclose(float(record[7]), 0.0, abs_tol=0.01)    # 0 rad → 0.00 deg

            # ends: ex, ey in metres; end_yaw in degrees
            assert float(record[8]) == 3.0
            assert float(record[9]) == 1.0
            assert math.isclose(float(record[10]), 90.0, abs_tol=0.01)  # pi/2 rad → 90.00 deg

            # initial errors: distance in cm, yaw in degrees (signed)
            # initial dist: 2.0 m → 200.00 cm
            assert math.isclose(float(record[11]), 200.0, abs_tol=0.01)
            # initial yaw: pi/2 rad → +90.00 deg  (sign preserved)
            assert math.isclose(float(record[12]), 90.0, abs_tol=0.01)

            # final errors: robot reached the goal → ~0 cm, ~0 deg
            assert math.isclose(float(record[13]), 0.0, abs_tol=0.01)
            assert math.isclose(float(record[14]), 0.0, abs_tol=0.01)

            # moving duration (seconds, unchanged)
            assert math.isclose(float(record[15]), 4.0)

    # Clean up temp file
    if os.path.exists(temp_csv_path):
        os.remove(temp_csv_path)
