import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.action import ActionClient
from std_msgs.msg import Empty, String, Float32, Bool
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

# [MODIFIED] Changed to robot_commander_two
from robot_commander_two.msg import ServiceRequest 
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
    - This node ONLY publishes and relays (Bridge layer)
    - Mode/Task decisions are managed by mode_manager
    """

    def __init__(self):
        super().__init__('ros2_mqtt_bridge')

        # [REMOVED] dock_x, dock_y, dock_yaw parameters deleted. Mode manager handles this.

        # Declare MQTT broker parameters
        self.declare_parameter('mqtt_host', 'localhost')
        self.declare_parameter('mqtt_port', 1883)

        self.mqtt_host = self.get_parameter('mqtt_host').value
        self.mqtt_port = self.get_parameter('mqtt_port').value

        # Initialize MQTT client
        self.mqtt_client = self._init_mqtt()
        
        # Initialize ROS subscriptions
        self._init_ros_subscribers()

        # Initialize ROS publishers
        # [MODIFIED] Removed leading slashes from all topic names
        self.service_request_pub = self.create_publisher(ServiceRequest, 'service_request', 10)
        self.cancel_pub = self.create_publisher(String, 'cancel_request', 10)
        self.wake_from_charge_pub = self.create_publisher(Empty, 'wake_from_charge', 10)
        self.pause_pub = self.create_publisher(Empty, 'pause', 10)
        self.resume_pub = self.create_publisher(Empty, 'resume', 10)
        self.gotodock_pub = self.create_publisher(Empty, 'gotodock', 10) # [ADDED] Replaces complex gotodock logic
        self.battery_pub = self.create_publisher(Float32, 'battery_level', 10)
        self.charging_status_pub = self.create_publisher(Bool, 'charging_status', 10) # [ADDED]
        
        # [REMOVED] activate_pub deleted.

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
        client = mqtt.Client(
            client_id="mqtt_bridge",
            protocol=mqtt.MQTTv311
        )
        client.on_message = self._on_mqtt_message
        
        # Connect to broker
        client.connect(self.mqtt_host, self.mqtt_port, 60)

        # Start background networking thread
        client.loop_start()

        # Subscribe to command and battery topics
        client.subscribe("robot/cmd/#", qos=1)
        client.subscribe("robot/battery/status", qos=0)
        client.subscribe("robot/powerswitch/event", qos=1) # [ADDED] New event topic

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
        # [MODIFIED] Removed leading slash
        self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self._pose_callback,
            10
        )

        # Service feedback (from mode_manager)
        #[MODIFIED] Removed leading slash
        self.create_subscription(
            String,
            'service_feedback',
            self._service_feedback_callback,
            10
        )

        # Robot state (from mode_manager)
        # [MODIFIED] Removed leading slash
        self.create_subscription(
            String,
            'robot_state',
            self._robot_state_callback,
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

        elif topic == "robot/cmd/gotodock":
            # [MODIFIED] Now just triggers a simple Empty message
            self._handle_simple_command(self.gotodock_pub, "GOTODOCK")

        elif topic == "robot/cmd/pause":
            self._handle_simple_command(self.pause_pub, "PAUSE")

        elif topic == "robot/cmd/resume":
            self._handle_simple_command(self.resume_pub, "RESUME")

        elif topic == "robot/cmd/wakeup":
            self._handle_simple_command(self.wake_from_charge_pub, "WAKEUP")

        # [REMOVED] elif topic == "robot/cmd/activate" completely removed

        elif topic == "robot/cmd/cancel_request":
            self._handle_cancel_request(payload)

        elif topic == "robot/battery/status":
            self._handle_mqtt_battery(payload)

        elif topic == "robot/powerswitch/event": # [ADDED] New event handler
            self._handle_powerswitch_event(payload)

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
            yaw = self._ensure_float(d["yaw"], "yaw")

            customer_id = d["customer_id"]
            timeout_sec = self._ensure_float(d["timeout_sec"], "timeout_sec")
            return_to_patrol_str = d["return_to_patrol"]
            priority = int(d["priority"])
            speed_limit_ms = self._ensure_float(d["speed_limit_ms"], "speed_limit_ms")

        except KeyError as e:
            self.get_logger().error(f"Missing field in service_request: {e}")
            return

        # Parse return_to_patrol as boolean
        if isinstance(return_to_patrol_str, str):
            return_to_patrol = return_to_patrol_str.lower() == "true"
        else:
            return_to_patrol = bool(return_to_patrol_str)

        msg = ServiceRequest()

        # ── PoseStamped ─────────────────────
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y

        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        msg.destination = pose

        # ── Metadata ────────────────────────
        msg.command_id = command_id
        msg.customer_id = customer_id
        msg.timeout_sec = float(timeout_sec)
        msg.return_to_patrol = bool(return_to_patrol)
        msg.priority = priority
        msg.speed_limit_ms = speed_limit_ms

        self.service_request_pub.publish(msg)

        self.get_logger().info(
            f"Service request → ({x:.2f}, {y:.2f}) priority={priority}"
        )

    def _handle_cancel_request(self, payload: dict):
        """
        Convert MQTT → ROS2 cancel_request
        """
        try:
            command_id = payload["command_id"]
        except KeyError as e:
            self.get_logger().error(f"Missing field in cancel request: {e}")
            return

        msg = String()
        msg.data = command_id
        self.cancel_pub.publish(msg)
        self.get_logger().info(f"Cancel request → {command_id}")

    # [REMOVED] _handle_gotodock function deleted entirely. 

    def _handle_mqtt_battery(self, payload: dict):
        """
        Convert MQTT battery data → ROS2 /battery_level
        """
        try:
            # Expected fields from battery controller: 
            # soc, voltage, current, power, ocv, cell_v, charging_state, charging, latch, soc_init, uptime_s
            battery_level = self._ensure_float(payload["soc"], "soc")
            self.get_logger().info(f"Forward field soc: {battery_level}")
        except KeyError as e:
            self.get_logger().error(f"Missing field in MQTT battery status: {e}")
            return
        except ValueError as e:
            self.get_logger().error(f"Invalid value in MQTT battery status: {e}")
            return

        # Publish battery level
        bat_msg = Float32()
        bat_msg.data = float(battery_level)
        self.battery_pub.publish(bat_msg)
        self.get_logger().debug(f"Battery level from MQTT: {battery_level:.2f}")

    def _handle_powerswitch_event(self, payload: dict):
        """
        Handle MQTT powerswitch events → ROS2 /charging_status
        """
        try:
            event_name = payload["event"]
        except KeyError as e:
            self.get_logger().error(f"Missing 'event' field in powerswitch event payload: {e}")
            return

        is_charging = False
        if event_name == "charger_connected":
            is_charging = True
        elif event_name == "charger_disconnected":
            is_charging = False
        else:
            self.get_logger().debug(f"Unhandled powerswitch event: {event_name}. Charging status not changed.")
            return # Don't publish if event is not related to charging

        # Publish charging status
        charge_msg = Bool()
        charge_msg.data = is_charging
        self.charging_status_pub.publish(charge_msg)
        self.get_logger().debug(f"Charging status from powerswitch event ({event_name}): {is_charging}")

    def _pose_callback(self, msg: PoseWithCovarianceStamped):
        """
        Handle incoming pose messages from ROS.
        Converts quaternion to yaw and publishes in your MQTT format.
        """
        pose = msg.pose.pose

        # Convert quaternion to yaw
        qz = pose.orientation.z
        qw = pose.orientation.w
        yaw = 2.0 * math.atan2(qz, qw)

        data = {
            "time": int(self._get_timestamp()),
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "yaw": float(yaw)
        }

        self._publish_mqtt("robot/state/pose", data)

    def _service_feedback_callback(self, msg: String):
        """
        Handle service feedback from mode_manager.
        """
        try:
            feedback = json.loads(msg.data)
            command_id = feedback.get("command_id", "unknown")
            status = feedback.get("status", "UNKNOWN")

            data = {
                "time": int(self._get_timestamp()),
                "command_id": command_id,
                "status": status
            }

            self._publish_mqtt("robot/state/service_feedback", data)

        except Exception as e:
            self.get_logger().error(f"Error parsing service_feedback: {e}")

    def _robot_state_callback(self, msg: String):
        """
        Handle robot state from mode_manager.
        Publishes current FSM state.
        """
        state = msg.data

        data = {
            "time": int(self._get_timestamp()),
            "state": state
        }

        self._publish_mqtt("robot/state/state", data)

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