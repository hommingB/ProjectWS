# service_robot — ROS2 Perception & Safety Package

A ROS2 (Humble) package for a 45×40 cm service robot running on **Raspberry Pi 5** with:
- **RPLIDAR A1M8** — mapping & nav costmap
- **Logitech C270** — person recognition
- **2× VL53L0X ToF** — front obstacle detection

---

## Package Structure

```
service_robot/
├── service_robot/
│   ├── tof_publisher_node.py          # Reads VL53L0X via I2C
│   ├── tof_safety_node.py             # cmd_vel obstacle safety filter
│   ├── tof_to_scan_bridge_node.py     # ToF → virtual LaserScan for costmap
│   └── person_detection_fusion_node.py # Camera + LiDAR person fusion
├── launch/
│   └── service_robot_launch.py
├── config/
│   └── tof_params.yaml
└── package.xml / setup.py
```

---

## Topic Graph

```
[VL53L0X I2C]
     │
     ▼
tof_publisher_node
  ├── /tof/left   (Range)
  ├── /tof/right  (Range)
  └── /tof/fused  (Range, min of both)
          │
          ├──► tof_safety_node ──── /cmd_vel_nav ──► /cmd_vel ──► wheel driver
          │         ▲ (Nav2 sends here)
          │
          └──► tof_to_scan_bridge_node ──► /tof/scan (LaserScan, 2-beam)
                                                │
                                           costmap2d obstacle_layer

[Logitech C270] ──► /camera/image_raw
[RPLIDAR A1M8]  ──► /scan
                        │
                        ▼
               person_detection_fusion_node
                  ├── /person_detection/detections       (Detection2DArray)
                  ├── /person_detection/target_bearing   (Float32, radians)
                  ├── /person_detection/target_goal      (PoseStamped → Nav2)
                  └── /person_detection/image_annotated  (Image, debug)
```

---

## Hardware Wiring

### VL53L0X I2C + XSHUT

Both sensors default to I2C address `0x29`. To run two on the same bus:

```
Pi 5 GPIO 17 ──► Left  VL53L0X XSHUT
Pi 5 GPIO 27 ──► Right VL53L0X XSHUT
SDA (GPIO 2)  ──► both sensors SDA
SCL (GPIO 3)  ──► both sensors SCL
3.3V / GND    ──► both sensors
```

The `tof_publisher_node` will:
1. Pull both XSHUT LOW (both sensors off)
2. Raise left XSHUT → assign address `0x30`
3. Raise right XSHUT → assign address `0x31`

Change `left_xshut_pin` / `right_xshut_pin` in `tof_params.yaml` to match your wiring.

### Sensor placement (from your diagram)

```
       FRONT (45 cm)
  ┌─────────────────────────┐
  │  [L]  ←18→ C ←18→ [R]  │   sensors 36 cm apart
  └─────────────────────────┘
   ↖55°             55°↗        beams angled outward
```

Beams converge ~20 cm ahead and cover ~54 cm at 80 cm range.

---

## Installation

### On Raspberry Pi 5

```bash
# Install Python libs
pip3 install VL53L0X RPi.GPIO

# Install ROS2 dependencies
sudo apt install ros-humble-cv-bridge ros-humble-vision-msgs \
                 ros-humble-sensor-msgs ros-humble-geometry-msgs

# Build
cd ~/ros2_ws
colcon build --packages-select service_robot
source install/setup.bash
```

### On your WSL2 / desktop (for vision node)

```bash
pip3 install opencv-python opencv-contrib-python
# Optional: for ONNX model
pip3 install onnxruntime
```

---

## Running

```bash
# Launch everything
ros2 launch service_robot service_robot_launch.py

# Override stop distance
ros2 launch service_robot service_robot_launch.py stop_dist_m:=0.25

# Disable debug image (saves CPU on Pi)
ros2 launch service_robot service_robot_launch.py publish_debug_img:=false
```

---

## Nav2 Integration

### 1. Route nav2 through safety node

In your Nav2 `controller_server` config, set:
```yaml
# nav2_params.yaml
controller_server:
  ros__parameters:
    cmd_vel_topic: /cmd_vel_nav   # ← nav2 publishes here
                                  #   safety node forwards to /cmd_vel
```

### 2. Add ToF virtual scan to local costmap

```yaml
# nav2_params.yaml
local_costmap:
  local_costmap:
    ros__parameters:
      observation_sources: scan tof_scan
      tof_scan:
        topic: /tof/scan
        sensor_frame: base_link
        data_type: LaserScan
        obstacle_max_range: 1.0
        obstacle_min_range: 0.0
        raytrace_max_range: 1.2
        raytrace_min_range: 0.0
        marking: true
        clearing: true
        inf_is_valid: false
```

### 3. Use person goal with Nav2 action client

The `/person_detection/target_goal` topic publishes `PoseStamped` in `base_link` frame.
A simple action client can forward this to the `NavigateToPose` action:

```python
# Minimal snippet
from nav2_msgs.action import NavigateToPose
# ... subscribe to /person_detection/target_goal
# ... transform to map frame with tf2
# ... send_goal(goal_pose)
```

---

## Simulation Mode

If `VL53L0X` / `RPi.GPIO` are not installed (e.g. on your WSL2 machine), the
`tof_publisher_node` will automatically enter **simulation mode** and publish
random distances in `[0.15, 1.10]` m — useful for testing the safety and
fusion nodes without hardware.

---

## Tuning Tips

| Parameter | Effect | Suggested range |
|-----------|--------|-----------------|
| `stop_dist_m` | Hard stop threshold | 0.15 – 0.25 m |
| `slow_dist_m` | Start braking distance | 0.35 – 0.60 m |
| `approach_dist_m` | Goal offset from person | 0.60 – 1.00 m |
| `publish_rate_hz` | ToF polling rate | 10 – 30 Hz |
| `camera_hfov_deg` | C270 horizontal FOV | ~60° |
| `left/right_sensor_bearing_deg` | Sensor angle from forward | match your mount |

---

## Upgrading the Person Detector

HOG is reliable but slow on Pi. Upgrade path:
1. Export `yolov8n.onnx` (person-only) from Ultralytics
2. Set `use_hog: false` and `onnx_model_path: /path/to/yolov8n.onnx`
3. Adapt `_detect_onnx()` output parsing to YOLOv8 format if needed
