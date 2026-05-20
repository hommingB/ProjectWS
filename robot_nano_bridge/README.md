# robot_nano_bridge — ROS2 ↔ Arduino Nano Bridge

ROS2 node that bridges ROS2 topics and MQTT messages to an Arduino Nano via serial communication for robot peripheral control (drawers and LEDs).

## File Structure

```
robot_nano_bridge/
├── __init__.py               # Package initialization
├── nano_interface.py         # Serial abstraction (protocol lives here ONLY)
├── state_translator.py       # ROS2/MQTT state → Nano command translation
├── robot_nano_bridge_node.py # ROS2 node entry-point
└── test_bridge.py            # Unit tests (no hardware needed)
```

## Layer Responsibilities

| File | Layer | Knows About |
|---|---|---|
| `nano_interface.py` | Serial I/O | Serial port only |
| `state_translator.py` | Business logic | NanoInterface API only |
| `robot_nano_bridge_node.py` | Integration | ROS2 + MQTT + both above |

## Quick Start

### 1. Install dependencies
```bash
pip install pyserial paho-mqtt
# ROS2 Humble/Iron must already be installed
```

### 2. Run tests (no hardware needed)
```bash
pip install pytest
pytest robot_nano_bridge/test_bridge.py -v
```

### 3. Build the package
```bash
cd ros2_project_ws
colcon build --packages-select robot_nano_bridge
source install/setup.bash
```

### 4. Run the node
```bash
# With default parameters
ros2 run robot_nano_bridge robot_nano_bridge_node

# Override serial port
ros2 run robot_nano_bridge robot_nano_bridge_node --ros-args \
  -p serial_port:=/dev/ttyUSB0 \
  -p serial_baud:=115200 \
  -p mqtt_host:=localhost \
  -p mqtt_port:=1883
```

## Topics and Parameters

### ROS2 Subscriptions
| Topic | Type | Description |
|---|---|---|
| `service_feedback` | `std_msgs/String` | JSON payload with command_id and status |
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands for motion LED indicators |

### MQTT Subscriptions
| Topic | Description |
|---|---|
| `robot/state/service_feedback` | Service feedback messages |
| `/robot/drawer/cmd` | Drawer control commands |

### Parameters
| Parameter | Default | Description |
|---|---|---|
| `serial_port` | `/dev/ttyUSB0` | Serial port for Nano connection |
| `serial_baud` | `115200` | Serial baud rate |
| `mqtt_host` | `localhost` | MQTT broker host |
| `mqtt_port` | `1883` | MQTT broker port |
| `mqtt_user` | `""` | MQTT username (optional) |
| `mqtt_password` | `""` | MQTT password (optional) |

## Serial Protocol (Pi → Nano)

| Command | Description |
|---|---|
| `DRV OPEN <id>` | Open drawer N |
| `DRV CLOSE <id>` | Close drawer N |
| `DRV HOME` | Home all drawers |
| `DRV STOP` | Stop drawer motor |
| `LED MODE PATROL` | Blue patrol mode |
| `LED MODE GUIDANCE` | Green guidance mode |
| `LED MODE DOCKING` | Amber docking mode |
| `LED MOTION FORWARD/REVERSE/LEFT/RIGHT/STOP` | Motion indicator |
| `LED OFF` | All LEDs off |
| `SYS PING` | Health check |
| `SYS ESTOP` | Emergency stop |

## Serial Protocol (Nano → Pi)

| Feedback | Meaning |
|---|---|
| `DRV DONE OPEN <id>` | Drawer opened successfully |
| `DRV DONE CLOSE <id>` | Drawer closed successfully |
| `DRV ERROR JAM <id>` | Drawer jammed |
| `DRV HOMED` | Homing complete |
| `SYS READY` | Nano boot complete |
| `SYS PONG` | Ping response |
| `LED ACK OFF` | LED off confirmed |

## LED Mode Mapping

| command_id Pattern | Mode | Color |
|---|---|---|
| `PATROL_*` | Patrol | Blue |
| `DOCK_*` | Docking | Amber |
| Any other ID | Guidance | Green |
| Error status | Error | LED OFF |

## Adding a New Command

Only edit `nano_interface.py`:
```python
def tilt_sensor(self, angle: int) -> None:
    self._send(f"TILT SET {angle}")
```

Then call it from `state_translator.py`:
```python
self._nano.tilt_sensor(45)
```

No other files need changing.