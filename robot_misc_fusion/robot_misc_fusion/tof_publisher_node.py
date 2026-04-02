#!/usr/bin/env python3
"""
ToF Publisher Node — VL53L0X via CH341T (USB→I2C) + TCA9548A mux
-----------------------------------------------------------------
Hardware chain:
  Pi 5 USB → CH341T → I2C bus → TCA9548A (0x70)
                                    ├── ch0 → VL53L0X left  (0x29)
                                    └── ch1 → VL53L0X right (0x29)

Both sensors keep their factory default address 0x29.
The TCA9548A mux selects which channel is active before each read,
so there is never a collision. No XSHUT wiring required.

The CH341T exposes as /dev/i2c-X — check with `ls /dev/i2c-*` and
`dmesg | grep ch341` after plugging in.  Typically i2c-0 or i2c-4.

Published topics:
  /tof/left   — sensor_msgs/Range
  /tof/right  — sensor_msgs/Range
  /tof/fused  — sensor_msgs/Range  (min of both, worst-case obstacle)

Parameters (tof_params.yaml):
  i2c_bus          : int   = 0       <- CH341T bus number
  mux_address      : int   = 0x70   <- TCA9548A default address
  left_mux_channel : int   = 0      <- mux channel for left sensor
  right_mux_channel: int   = 1      <- mux channel for right sensor
  sensor_address   : int   = 0x29   <- VL53L0X default (same for both)
  publish_rate_hz  : float = 20.0
  frame_id_left    : str   = "tof_left"
  frame_id_right   : str   = "tof_right"
  fov_rad          : float = 0.436
  min_range_m      : float = 0.03
  max_range_m      : float = 1.20
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Header
import time

try:
    import smbus2
    SMBUS_OK = True
except ImportError:
    SMBUS_OK = False

try:
    import VL53L0X
    TOF_OK = True
except ImportError:
    TOF_OK = False

HW_AVAILABLE = SMBUS_OK and TOF_OK

if not HW_AVAILABLE:
    import random


class TCA9548A:
    """Minimal TCA9548A I2C mux driver."""

    def __init__(self, bus, address: int = 0x70):
        self._bus  = bus
        self._addr = address

    def select(self, channel: int):
        if not 0 <= channel <= 7:
            raise ValueError(f'Channel must be 0-7, got {channel}')
        self._bus.write_byte(self._addr, 1 << channel)
        time.sleep(0.001)

    def disable_all(self):
        self._bus.write_byte(self._addr, 0x00)


class TofPublisherNode(Node):

    def __init__(self):
        super().__init__('tof_publisher_node')

        self.declare_parameter('i2c_bus',           0)
        self.declare_parameter('mux_address',       0x70)
        self.declare_parameter('left_mux_channel',  0)
        self.declare_parameter('right_mux_channel', 1)
        self.declare_parameter('sensor_address',    0x29)
        self.declare_parameter('publish_rate_hz',   20.0)
        self.declare_parameter('frame_id_left',     'tof_left')
        self.declare_parameter('frame_id_right',    'tof_right')
        self.declare_parameter('fov_rad',           0.436)
        self.declare_parameter('min_range_m',       0.03)
        self.declare_parameter('max_range_m',       1.20)

        self.i2c_bus_num = self.get_parameter('i2c_bus').value
        self.mux_addr    = self.get_parameter('mux_address').value
        self.ch_left     = self.get_parameter('left_mux_channel').value
        self.ch_right    = self.get_parameter('right_mux_channel').value
        self.sensor_addr = self.get_parameter('sensor_address').value
        rate_hz          = self.get_parameter('publish_rate_hz').value
        self.frame_left  = self.get_parameter('frame_id_left').value
        self.frame_right = self.get_parameter('frame_id_right').value
        self.fov_rad     = self.get_parameter('fov_rad').value
        self.min_range   = self.get_parameter('min_range_m').value
        self.max_range   = self.get_parameter('max_range_m').value

        self.pub_left  = self.create_publisher(Range, '/tof/left',  10)
        self.pub_right = self.create_publisher(Range, '/tof/right', 10)
        self.pub_fused = self.create_publisher(Range, '/tof/fused', 10)

        self.mux    = None
        self.sensor = None
        self._bus   = None

        if HW_AVAILABLE:
            self._init_hardware()
        else:
            missing = ([] if SMBUS_OK else ['smbus2']) + ([] if TOF_OK else ['VL53L0X'])
            self.get_logger().warn(f'Missing: {missing} — SIMULATION mode')

        self.create_timer(1.0 / rate_hz, self._timer_cb)
        self.get_logger().info(
            f'ToF publisher ready @ {rate_hz} Hz  hw={HW_AVAILABLE}  '
            f'bus=i2c-{self.i2c_bus_num}  mux=0x{self.mux_addr:02X}')

    def _init_hardware(self):
        self._bus = smbus2.SMBus(self.i2c_bus_num)
        self.mux  = TCA9548A(self._bus, self.mux_addr)
        self.mux.disable_all()
        self.get_logger().info(
            f'TCA9548A on i2c-{self.i2c_bus_num} @ 0x{self.mux_addr:02X}')

        # Single VL53L0X instance — mux channel swapped before each read
        self.sensor = VL53L0X.VL53L0X(
            i2c_bus=self.i2c_bus_num,
            i2c_address=self.sensor_addr)
        self.sensor.open()

        for ch, side in [(self.ch_left, 'left'), (self.ch_right, 'right')]:
            self.mux.select(ch)
            self.sensor.start_ranging(VL53L0X.Vl53l0xAccuracyMode.HIGH_SPEED)
            mm = self.sensor.get_distance()
            self.sensor.stop_ranging()
            self.get_logger().info(f'  ch{ch} ({side}): {mm} mm')

        self.mux.disable_all()
        self.get_logger().info('Both sensors OK')

    def _read_channel_m(self, channel: int) -> float:
        self.mux.select(channel)
        self.sensor.start_ranging(VL53L0X.Vl53l0xAccuracyMode.HIGH_SPEED)
        mm = self.sensor.get_distance()
        self.sensor.stop_ranging()
        self.mux.disable_all()
        if mm <= 0:
            return self.max_range
        return max(self.min_range, min(self.max_range, mm / 1000.0))

    def _sim_distance(self) -> float:
        return round(random.uniform(0.15, 1.10), 3)

    def _make_range_msg(self, frame_id: str, distance_m: float) -> Range:
        msg = Range()
        msg.header             = Header()
        msg.header.stamp       = self.get_clock().now().to_msg()
        msg.header.frame_id    = frame_id
        msg.radiation_type     = Range.INFRARED
        msg.field_of_view      = self.fov_rad
        msg.min_range          = self.min_range
        msg.max_range          = self.max_range
        msg.range              = float(distance_m)
        return msg

    def _timer_cb(self):
        if HW_AVAILABLE and self.mux and self.sensor:
            try:
                d_left  = self._read_channel_m(self.ch_left)
                d_right = self._read_channel_m(self.ch_right)
            except Exception as e:
                self.get_logger().warn(f'ToF read error: {e}',
                                       throttle_duration_sec=2.0)
                d_left = d_right = self.max_range
        else:
            d_left  = self._sim_distance()
            d_right = self._sim_distance()

        self.pub_left.publish(self._make_range_msg(self.frame_left,  d_left))
        self.pub_right.publish(self._make_range_msg(self.frame_right, d_right))
        self.pub_fused.publish(
            self._make_range_msg('tof_center', min(d_left, d_right)))

    def destroy_node(self):
        if HW_AVAILABLE:
            try:
                if self.mux:    self.mux.disable_all()
                if self.sensor: self.sensor.close()
                if self._bus:   self._bus.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TofPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
