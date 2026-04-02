import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import BatteryState
from rclpy.action import ActionClient
from std_msgs.msg import Empty
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from robot_commander.msg import ServiceRequest
import math

import json
import signal
import threading
import paho.mqtt.client as mqtt


class Ros2MqttBridge(Node):
    """
    ROS2 → MQTT Bridge Node

    Responsibilities:
    - Subscribe to ROS2 topics (pose, battery)
    - Transform data into structured JSON
    - Publish to MQTT broker

    Notes:
    - This node ONLY publishes (Stage 1)
    - Command handling (MQTT → ROS2) will be added later
    """

    def __init__(self):
        super().__init__('ros2_mqtt_bridge')

        # Initialize MQTT client
        self.mqtt_client = self._init_mqtt()
        
        # Initialize ROS subscriptions
        self._init_ros_subscribers()

        # Initialize ROS publishers
        self.service_request_pub = self.create_publisher(
            ServiceRequest,
            '/service_request',
            10
        )
        

        self.pause_pub = self.create_publisher(Empty, '/pause', 10)
        self.resume_pub = self.create_publisher(Empty, '/resume', 10)
        self.activate_pub = self.create_publisher(Empty, '/activate', 10)

        self.get_logger().info("ROS2 MQTT Bridge started")

    # =========================================================
    # 🔌 MQTT SETUP
    # =========================================================

    def _init_mqtt(self):
        """
        Initialize and connect MQTT client.

        Returns:
            mqtt.Client: connected MQTT client instance
        """
        client = mqtt.Client()
        client.on_message = self._on_mqtt_message
        
        # Connect to broker (adjust IP if needed)
        client.connect("localhost", 1883, 60)

        # Start background networking thread
        client.loop_start()

        # Subscribe to command topic
        client.subscribe("robot/cmd/#", qos=1)

        return client

    def _publish_mqtt(self, topic: str, data: dict):
        """
        Publish a structured message to MQTT.

        Args:
            topic (str): MQTT topic
            data (dict): payload data
        """
        message = {
            "ts": self._get_timestamp(),
            "source": "robot_1",
            "data": data
        }

        payload = json.dumps(message)

        # QoS=1 ensures delivery (at least once)
        self.mqtt_client.publish(topic, payload, qos=1)

    # =========================================================
    # ⏱️ TIME HANDLING
    # =========================================================

    def _get_timestamp(self) -> float:
        """
        Get current ROS time as UNIX timestamp.

        Returns:
            float: timestamp in seconds
        """
        now = self.get_clock().now().to_msg()
        return now.sec + now.nanosec * 1e-9

    # =========================================================
    # 🤖 ROS2 SETUP
    # =========================================================

    def _init_ros_subscribers(self):
        """
        Initialize all ROS2 topic subscriptions.
        """
        # Pose (from AMCL)
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._pose_callback,
            10
        )

        # Battery state
        self.create_subscription(
            BatteryState,
            '/battery_state',
            self._battery_callback,
            10
        )

    # =========================================================
    # 📡 ROS CALLBACKS
    # =========================================================
    def _on_mqtt_message(self, client, userdata, msg):
        """
        Handle incoming MQTT messages
        """
        topic = msg.topic

        try:
            payload = json.loads(msg.payload.decode())
        except Exception as e:
            self.get_logger().error(f"Invalid JSON: {e}")
            return

        if topic == "robot/cmd/service_request":
            self._handle_service_request(payload)

        elif topic == "robot/cmd/pause":
            self._handle_simple_command(self.pause_pub, "PAUSE")

        elif topic == "robot/cmd/resume":
            self._handle_simple_command(self.resume_pub, "RESUME")

        elif topic == "robot/cmd/activate":
            self._handle_simple_command(self.activate_pub, "ACTIVATE")

    def _handle_simple_command(self, publisher, name: str):
        """
        Publish Empty command to ROS2
        """
        publisher.publish(Empty())
        self.get_logger().info(f"{name} command received from MQTT")

    def _handle_service_request(self, payload: dict):
        """
        Convert MQTT → ROS2 ServiceRequest
        """

        try:
            d = payload["data"]
            command_id = payload.get("command_id", "unknown")
            
            x = self._ensure_float(d["x"], "x")
            y = self._ensure_float(d["y"], "y")
            theta = self._ensure_float(d["theta"], "theta")

            customer_id = d["customer_id"]
            timeout_sec = d["timeout_sec"]
            return_to_patrol = d["return_to_patrol"]
            priority = d["priority"]
            halt_on_arrival = d["halt_on_arrival"]

        except KeyError as e:
            self.get_logger().error(f"Missing field: {e}")
            return

        msg = ServiceRequest()

        # ── PoseStamped ─────────────────────
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y

        qz = math.sin(theta / 2.0)
        qw = math.cos(theta / 2.0)

        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        msg.destination = pose

        # ── Metadata ────────────────────────
        msg.customer_id = customer_id
        msg.timeout_sec = float(timeout_sec)
        msg.return_to_patrol = bool(return_to_patrol)
        msg.priority = int(priority)
        msg.halt_on_arrival = bool(halt_on_arrival)

        self.service_request_pub.publish(msg)

        self.get_logger().info(
            f"Service request → ({x:.2f}, {y:.2f}) priority={priority}"
        )

    def _pose_callback(self, msg: PoseWithCovarianceStamped):
        """
        Handle incoming pose messages from ROS.

        Args:
            msg: PoseWithCovarianceStamped message
        """
        pose = msg.pose.pose

        data = {
            "x": pose.position.x,
            "y": pose.position.y,
            "z": pose.position.z,
            "qx": pose.orientation.x,
            "qy": pose.orientation.y,
            "qz": pose.orientation.z,
            "qw": pose.orientation.w
        }

        self._publish_mqtt("robot/state/pose", data)

    def _battery_callback(self, msg: BatteryState):
        """
        Handle incoming battery messages from ROS.

        Args:
            msg: BatteryState message
        """
        data = {
            "voltage": msg.voltage,
            "percentage": msg.percentage
        }

        self._publish_mqtt("robot/state/battery", data)

    # =========================================================
    # (1) HELPER
    # =========================================================
    
    def _ensure_float(self, value, name):
        try:
            return float(value)
        except Exception:
            raise ValueError(f"{name} must be numeric")

    # =========================================================
    # 🧹 CLEANUP
    # =========================================================

    def cleanup(self):
        """
        Gracefully shutdown MQTT connection.
        """
        self.get_logger().info("Cleaning up MQTT connection...")

        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()


# =========================================================
# 🚀 MAIN ENTRY POINT
# =========================================================

def main(args=None):
    rclpy.init(args=args)
    node = Ros2MqttBridge()

    shutdown_event = threading.Event()

    def _signal_handler(signum, frame):
        node.get_logger().info(f"Signal received ({signum}), shutting down gracefully...")
        shutdown_event.set()

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    try:
        node.get_logger().info("Entering spin loop")
        while rclpy.ok() and not shutdown_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)

    except Exception as e:
        node.get_logger().error(f"Unexpected error in spin loop: {e}")

    finally:
        node.get_logger().info("Shutting down node")
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()