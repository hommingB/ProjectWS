# Robot Nano Bridge Node Summary

This ROS2 node bridges ROS2 topics and MQTT messages to an Arduino Nano via serial communication. It provides fault tolerance and handles reconnections for serial and MQTT.

## Features:

*   **Serial Communication:**
    *   Uses `NanoInterface` for auto-reconnecting serial communication with exponential backoff.
    *   Configurable serial port and baud rate.
*   **MQTT Bridging (Optional):**
    *   Connects to an MQTT broker to exchange messages.
    *   Watchdog timer to detect and recover from silent disconnects.
    *   Supports user/password authentication.
     *   Subscribes to `service_feedback` ROS2 topic.

### MQTT Topics

The node (when MQTT is enabled) subscribes to the following MQTT topics:

* `robot/state/service_feedback`
* `/robot/drawer/cmd`

These topics carry JSON‑encoded messages that are forwarded to the `StateTranslator` for handling.
*   **ROS2 Integration:**
    *   Subscribes to `service_feedback` ROS2 topic.
    *   Uses `StateTranslator` to translate ROS2 messages into Nano serial commands.
*   **Fault Tolerance:**
    *   Wraps callbacks in `try/except` blocks to prevent crashes.
    *   Handles serial and MQTT connection failures gracefully.
*   **Parameters:**
    *   `serial_port`: Serial port for Nano communication (default: `/NANO_hub.2`).
    *   `serial_baud`: Serial baud rate (default: 9600).
    *   `mqtt_host`: MQTT broker host (default: `localhost`).
    *   `mqtt_port`: MQTT broker port (default: 1883).
    *   `mqtt_user`: MQTT username (default: empty).
    *   `mqtt_password`: MQTT password (default: empty).

## Protocol:

The node uses a JSON-based protocol for both ROS2 and MQTT messages. The `StateTranslator` class handles the translation between these messages and the serial commands sent to the Nano.

## QoS:

The ROS2 subscriptions use the default QoS settings.

## Usage:

Run the node with:

```bash
ros2 run robot_nano_bridge robot_nano_bridge_node
```

## Notes:

*   MQTT is optional and can be disabled in test environments.
*   The `cmd_vel` ROS2 topic is disabled.
*   The node uses a background thread for MQTT connection management.
*   The node provides logging for debugging and monitoring.

## Dependencies:

*   `rclpy`
*   `std_msgs`
*   `paho-mqtt` (optional)
*   `robot_nano_bridge` (custom modules)
