# test_robot_bridge_node.py
import json
import pytest
from unittest.mock import MagicMock, patch

import rclpy
from std_msgs.msg import String

# ----------------------------------------------------------------------
# Fake NanoInterface – records calls for assertions
# ----------------------------------------------------------------------
class FakeNano:
    def __init__(self):
        self.calls = []
    def set_led_mode(self, mode):  self.calls.append(("set_led_mode", mode))
    def led_off(self):            self.calls.append(("led_off", None))
    # Drawer methods (no‑ops for this test)
    def open_drawer(self, _): pass
    def close_drawer(self, _): pass
    def home_drawer(self): pass
    def stop_drawer(self): pass
    def connect(self): pass
    def disconnect(self): pass

# ----------------------------------------------------------------------
@pytest.fixture(scope="function")
def rclpy_init_shutdown():
    rclpy.init()
    yield
    rclpy.shutdown()

def test_service_feedback_flow(rclpy_init_shutdown, monkeypatch):
    fake_nano = FakeNano()
    # Patch NanoInterface constructor to return our fake instance
    monkeypatch.setattr(
        "robot_nano_bridge.nano_interface.NanoInterface",
        lambda *a, **kw: fake_nano
    )

    # Import after patch so the node uses FakeNano
    from robot_nano_bridge.robot_nano_bridge_node import RobotBridgeNode

    node = RobotBridgeNode()
    exec = rclpy.executors.SingleThreadedExecutor()
    exec.add_node(node)

    # Publisher for service_feedback
    pub = node.create_publisher(String, "service_feedback", 10)
    msg = String()
    payload = {"command_id": "PATROL_01", "status": "EXECUTING"}
    msg.data = json.dumps(payload)

    # Publish and spin a few cycles
    pub.publish(msg)
    for _ in range(5):
        exec.spin_once(timeout_sec=0.1)

    # The translator should have called set_led_mode with the PATROL constant
    assert any(call[0] == "set_led_mode" for call in fake_nano.calls), \
        "No LED mode call recorded"
    mode_arg = next(call[1] for call in fake_nano.calls if call[0] == "set_led_mode")
    # The exact constant lives in nano_interface – import to compare
    from robot_nano_bridge.nano_interface import LED_MODE_PATROL
    assert mode_arg == LED_MODE_PATROL, f"Expected PATROL mode, got {mode_arg}"

    node.destroy_node()