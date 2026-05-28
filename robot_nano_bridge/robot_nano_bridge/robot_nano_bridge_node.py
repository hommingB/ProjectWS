"""
robot_bridge_node.py
--------------------
ROS2 node that bridges ROS2 topics + MQTT → Arduino Nano serial commands.

Fault tolerance added
---------------------
- Serial: NanoInterface auto-reconnects with exponential back-off (see nano_interface.py).
- MQTT:   A watchdog timer periodically checks the connection and triggers
          reconnect if the broker went away. paho's own reconnect is also
          enabled as a first line of defence.
- ROS2 callbacks: every callback is wrapped in try/except so a bad message
          never crashes the node.
- Node init: serial and MQTT failures are logged but never raise — the node
          always finishes __init__ and starts spinning.
- Shutdown: destroy_node() is safe even if serial / MQTT were never connected.

Run with:
    ros2 run robot_bridge robot_bridge_node

Or directly (for testing without colcon):
    python3 robot_bridge_node.py
"""

import json
import logging
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# Optional MQTT — graceful degradation if paho not installed
try:
    import paho.mqtt.client as mqtt
    MQTT_AVAILABLE = True
except ImportError:
    MQTT_AVAILABLE = False

from robot_nano_bridge.nano_interface import NanoInterface
from robot_nano_bridge.state_translator import StateTranslator

# ── Logging ───────────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger("robot_bridge")

# ── MQTT retry tuning ─────────────────────────────────────────────────────────
MQTT_INITIAL_RETRY  = 2.0    # seconds
MQTT_MAX_RETRY      = 60.0   # cap
MQTT_BACKOFF        = 2.0
MQTT_WATCHDOG_SEC   = 15.0   # how often the watchdog checks the connection


class RobotBridgeNode(Node):
    """
    Main ROS2 node.  All subscriber callbacks delegate to StateTranslator,
    which calls NanoInterface — no raw serial strings anywhere in this file.
    """

    def __init__(self):
        super().__init__("robot_bridge_node")

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter("serial_port",   "/dev/NANO_hub.2")
        self.declare_parameter("serial_baud",   9600)
        self.declare_parameter("mqtt_host",     "localhost")
        self.declare_parameter("mqtt_port",     1883)
        self.declare_parameter("mqtt_user",     "")
        self.declare_parameter("mqtt_password", "")

        serial_port = self.get_parameter("serial_port").value
        serial_baud = self.get_parameter("serial_baud").value

        # ── Nano serial interface (auto-reconnects internally) ────────────────
        self._nano = NanoInterface(
            port=serial_port,
            baud=serial_baud,
            feedback_callback=self._on_nano_feedback,
        )
        # connect() starts background threads and returns immediately — never raises
        self._nano.connect()

        # ── State translator (pure logic, no I/O) ─────────────────────────────
        self._translator = StateTranslator(
            self._nano,
            on_state_update_cb=self._publish_mqtt
        )

        # ── ROS2 subscribers ──────────────────────────────────────────────────
        # Service feedback subscription (kept) – cmd_vel removed
        self.create_subscription(
            String,
            "service_feedback",
            self._ros_service_feedback_cb,
            10,
        )
        logger.info("ROS2 subscribers ready (service_feedback only)")

        # ── MQTT (optional) ───────────────────────────────────────────────────
        self._mqtt_client:       object = None
        self._mqtt_connected:    bool   = False
        self._mqtt_retry_delay:  float  = MQTT_INITIAL_RETRY
        self._mqtt_running:      bool   = False
        self._mqtt_lock                 = threading.Lock()

        # MQTT optional – disabled in test environments to avoid connection attempts
        if MQTT_AVAILABLE:
            self._mqtt_running = True
            self._start_mqtt_with_retry()
            # Watchdog timer — fires every MQTT_WATCHDOG_SEC seconds
            self.create_timer(MQTT_WATCHDOG_SEC, self._mqtt_watchdog)
        else:
            logger.info("MQTT bridge disabled (test environment)")

        logger.info("RobotBridgeNode started (serial=%s)", serial_port)

    # ── ROS2 callbacks ────────────────────────────────────────────────────────
    def _ros_cmd_vel_cb(self, msg):
        # cmd_vel callback disabled; no action taken.
        logger.debug("Ignored cmd_vel message (disabled).")

    def _ros_service_feedback_cb(self, msg: String) -> None:
        """Handle service_feedback JSON string and forward to translator.
        This method is kept for tests; it simply parses the msg.data.
        """
        try:
            payload = json.loads(msg.data)
            self._translator.on_service_feedback(payload)
        except json.JSONDecodeError as exc:
            logger.warning("Malformed service_feedback JSON: %s — %s", msg.data, exc)
        except Exception:
            logger.exception("Unexpected error in service_feedback callback")

    # ── Nano feedback ─────────────────────────────────────────────────────────
    def _on_nano_feedback(self, line: str) -> None:
        """Runs in the serial reader thread — keep it fast, never raise."""
        try:
            self._translator.on_nano_feedback(line)
        except Exception:
            logger.exception("Error handling Nano feedback: %s", line)

    # ── MQTT: initial connect with retry ─────────────────────────────────────
    def _start_mqtt_with_retry(self) -> None:
        """Spawn a one-shot thread that keeps trying until MQTT connects."""
        t = threading.Thread(
            target=self._mqtt_connect_loop, daemon=True, name="mqtt-connect"
        )
        t.start()

    def _mqtt_connect_loop(self) -> None:
        """Background thread: attempt MQTT connection with exponential back-off."""
        host     = self.get_parameter("mqtt_host").value
        port     = self.get_parameter("mqtt_port").value
        user     = self.get_parameter("mqtt_user").value
        password = self.get_parameter("mqtt_password").value

        while self._mqtt_running and not self._mqtt_connected:
            try:
                logger.info("Attempting MQTT connection to %s:%d …", host, port)
                client = mqtt.Client(
                    client_id="robot_bridge_node",
                    protocol=mqtt.MQTTv311
                )
                if user:
                    client.username_pw_set(user, password)

                client.on_connect    = self._mqtt_on_connect
                client.on_message    = self._mqtt_on_message
                client.on_disconnect = self._mqtt_on_disconnect

                # paho built-in reconnect as a secondary safety net
                client.reconnect_delay_set(min_delay=1, max_delay=30)

                client.connect(host, port, keepalive=60)

                with self._mqtt_lock:
                    self._mqtt_client = client

                # loop_forever blocks; on_connect will set _mqtt_connected=True
                client.loop_forever()

                # loop_forever returns when disconnect() is called or on fatal error
                if self._mqtt_running:
                    logger.warning("MQTT loop_forever exited — will reconnect")

            except (ConnectionRefusedError, OSError) as exc:
                logger.warning(
                    "MQTT connect to %s:%d failed: %s — retry in %.0f s",
                    host, port, exc, self._mqtt_retry_delay,
                )
            except Exception:
                logger.exception(
                    "Unexpected MQTT error — retry in %.0f s", self._mqtt_retry_delay
                )
            finally:
                self._mqtt_connected = False

            # Back-off sleep (interruptible)
            end = time.monotonic() + self._mqtt_retry_delay
            while self._mqtt_running and not self._mqtt_connected and time.monotonic() < end:
                time.sleep(0.5)

            self._mqtt_retry_delay = min(
                self._mqtt_retry_delay * MQTT_BACKOFF, MQTT_MAX_RETRY
            )

        logger.debug("MQTT connect loop exiting")

    # ── MQTT: watchdog ────────────────────────────────────────────────────────
    def _mqtt_watchdog(self) -> None:
        """Called every MQTT_WATCHDOG_SEC seconds to detect silent disconnects."""
        if not MQTT_AVAILABLE or not self._mqtt_running:
            return
        if not self._mqtt_connected:
            logger.warning("MQTT watchdog: not connected — starting reconnect loop")
            self._mqtt_retry_delay = MQTT_INITIAL_RETRY   # reset back-off
            self._start_mqtt_with_retry()

    # ── MQTT: paho callbacks ──────────────────────────────────────────────────
    def _mqtt_on_connect(self, client, userdata, flags, rc) -> None:
        if rc == 0:
            self._mqtt_connected   = True
            self._mqtt_retry_delay = MQTT_INITIAL_RETRY   # reset back-off
            logger.info("MQTT connected — subscribing to topics")
            client.subscribe("robot/state/service_feedback")
            client.subscribe("robot/drawer/cmd")
        else:
            logger.error("MQTT broker refused connection, rc=%d", rc)

    def _mqtt_on_disconnect(self, client, userdata, rc) -> None:
        self._mqtt_connected = False
        if rc == 0:
            logger.info("MQTT disconnected cleanly")
        else:
            logger.warning(
                "MQTT disconnected unexpectedly (rc=%d) — paho will auto-reconnect", rc
            )

    def _mqtt_on_message(self, client, userdata, msg) -> None:
        topic = msg.topic
        try:
            payload_str = msg.payload.decode("utf-8", errors="replace")
            payload = json.loads(payload_str)
        except json.JSONDecodeError:
            logger.warning("Non-JSON MQTT message on %s: %s", topic, msg.payload)
            return
        except Exception:
            logger.exception("Error decoding MQTT message on %s", topic)
            return

        logger.debug("MQTT ← [%s] %s", topic, payload)
        try:
            if topic == "robot/state/service_feedback":
                self._translator.on_service_feedback(payload)
            elif topic in ("robot/drawer/cmd", "/robot/drawer/cmd"):
                self._translator.on_drawer_cmd(payload)
            else:
                logger.debug("Unhandled MQTT topic: %s", topic)
        except Exception:
            logger.exception("Error handling MQTT message on %s: %s", topic, payload)

    def _publish_mqtt(self, topic: str, payload: dict) -> None:
        """Publish a JSON payload to an MQTT topic safely."""
        if not MQTT_AVAILABLE or not self._mqtt_running:
            return
        with self._mqtt_lock:
            client = self._mqtt_client
        if client and self._mqtt_connected:
            try:
                payload_str = json.dumps(payload)
                client.publish(topic, payload_str)
                logger.info("Published MQTT → [%s] %s", topic, payload_str)
            except Exception as exc:
                logger.error("Failed to publish MQTT on %s: %s", topic, exc)

    # ── Cleanup ───────────────────────────────────────────────────────────────
    def destroy_node(self) -> None:
        logger.info("Shutting down RobotBridgeNode…")

        # Stop MQTT
        self._mqtt_running    = False
        self._mqtt_connected  = False
        with self._mqtt_lock:
            client = self._mqtt_client
        if client:
            try:
                client.disconnect()
            except Exception:
                pass

        # Safe LED off before serial closes
        try:
            self._nano.led_off()
        except Exception:
            pass

        self._nano.disconnect()
        super().destroy_node()


# ── Entry point ───────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = RobotBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()