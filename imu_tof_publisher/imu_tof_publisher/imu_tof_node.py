#!/usr/bin/env python3
"""
imu_tof_node.py
---------------
ROS2 node for reading BNO085 IMU (CH7) and two VL53L0X ToF sensors
(CH5=left, CH6=right) through a TCA9548A I2C multiplexer.

Topics published:
  /imu/data          → sensor_msgs/Imu                  (50 Hz)
  /tof/left          → sensor_msgs/Range                 (20 Hz)
  /tof/right         → sensor_msgs/Range                 (20 Hz)
  /diagnostics       → diagnostic_msgs/DiagnosticArray   (1 Hz)

Fault tolerance:
  - Sensors missing at boot  → node starts anyway, marks sensor absent
  - Sensors lost at runtime  → consecutive-failure counter triggers reconnect
  - All sensors missing      → node still runs; /diagnostics reports full failure

Author: generated for Raspberry Pi 5 + ROS2 Humble/Iron
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

import math
import time
import collections

from sensor_msgs.msg import Imu, Range
from std_msgs.msg import Header
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

# ── I2C / sensor libraries ──────────────────────────────────────────────────
import board
import busio
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C
import adafruit_vl53l0x

# ── Constants ────────────────────────────────────────────────────────────────
MUX_ADDR        = 0x70          # TCA9548A default address
MUX_CH_TOF_LEFT  = 5           # CH5 → left  VL53L0X
MUX_CH_TOF_RIGHT = 6           # CH6 → right VL53L0X
MUX_CH_IMU       = 7           # CH7 → BNO085

BNO085_ADDR     = 0x4B         # BNO085 I2C address (0x4B if ADR pin high)
VL53L0X_ADDR    = 0x29         # Default VL53L0X address (same for both, mux isolates)

VL53_MIN_RANGE  = 0.03         # metres
VL53_MAX_RANGE  = 1.20         # metres  (practical indoor limit for your setup)
VL53_FOV_RAD    = math.radians(27.0)   # VL53L0X full cone FOV

MEDIAN_WINDOW   = 5            # samples for median filter
EMA_ALPHA       = 0.3          # exponential moving average weight (0=heavy smooth, 1=raw)

IMU_RATE_HZ     = 50.0
TOF_RATE_HZ     = 20.0
DIAG_RATE_HZ    = 1.0

# How many consecutive callback failures before a reconnect is attempted
RECONNECT_AFTER = 10


# ── Utility: Median + EMA filter ─────────────────────────────────────────────
class RangeFilter:
    """Rolling median spike rejection followed by exponential moving average."""

    def __init__(self, window: int = MEDIAN_WINDOW, alpha: float = EMA_ALPHA):
        self.window  = window
        self.alpha   = alpha
        self._buf    = collections.deque(maxlen=window)
        self._ema    = None                 # initialised on first valid reading

    def update(self, raw_m: float) -> float | None:
        """
        Feed a new raw distance (metres).
        Returns filtered distance, or None if the buffer isn't full yet
        or the reading is out of sensor range.
        """
        if raw_m < VL53_MIN_RANGE or raw_m > VL53_MAX_RANGE:
            return None                     # discard invalid / out-of-range

        self._buf.append(raw_m)
        if len(self._buf) < self.window:
            return None                     # wait for buffer to fill

        median = sorted(self._buf)[self.window // 2]

        if self._ema is None:
            self._ema = median
        else:
            self._ema = self.alpha * median + (1.0 - self.alpha) * self._ema

        return self._ema


# ── Main Node ────────────────────────────────────────────────────────────────
class ImuTofNode(Node):

    def __init__(self):
        super().__init__('imu_tof_node')

        # ── ROS2 parameters (tune without recompiling) ─────────────────────
        self.declare_parameter('mux_address',       MUX_ADDR)
        self.declare_parameter('imu_rate_hz',        IMU_RATE_HZ)
        self.declare_parameter('tof_rate_hz',        TOF_RATE_HZ)
        self.declare_parameter('median_window',      MEDIAN_WINDOW)
        self.declare_parameter('ema_alpha',          EMA_ALPHA)
        self.declare_parameter('imu_frame_id',       'imu_link')
        self.declare_parameter('tof_left_frame_id',  'tof_left_link')
        self.declare_parameter('tof_right_frame_id', 'tof_right_link')
        # Toe-in geometry — used only for informational logging here;
        # actual ray direction comes from TF/URDF
        self.declare_parameter('tof_toe_in_deg',    11.3)
        self.declare_parameter('crossover_dist_m',   0.40)

        p = self.get_parameters([
            'mux_address', 'imu_rate_hz', 'tof_rate_hz',
            'median_window', 'ema_alpha',
            'imu_frame_id', 'tof_left_frame_id', 'tof_right_frame_id',
            'tof_toe_in_deg', 'crossover_dist_m',
        ])
        (self._mux_addr, imu_hz, tof_hz,
         med_win, ema_a,
         self._imu_fid, self._tof_l_fid, self._tof_r_fid,
         toe_deg, cross_m) = [x.value for x in p]

        self.get_logger().info(
            f"ToF toe-in: {toe_deg:.1f}°  |  crossover at {cross_m*100:.0f} cm"
        )

        # ── I2C bus ────────────────────────────────────────────────────────
        self._i2c = busio.I2C(board.SCL, board.SDA)

        # ── Health tracking ────────────────────────────────────────────────
        # Each entry: {'sensor': obj|None, 'healthy': bool, 'fails': int}
        self._imu_state = self._make_state()
        self._tof_l_state = self._make_state()
        self._tof_r_state = self._make_state()

        # ── Fault-tolerant sensor init ─────────────────────────────────────
        self._imu_state['sensor']  = self._try_init_imu()
        self._tof_l_state['sensor'] = self._try_init_tof(MUX_CH_TOF_LEFT,  'left')
        self._tof_r_state['sensor'] = self._try_init_tof(MUX_CH_TOF_RIGHT, 'right')

        # ── Filters ────────────────────────────────────────────────────────
        self._filt_left  = RangeFilter(window=int(med_win), alpha=float(ema_a))
        self._filt_right = RangeFilter(window=int(med_win), alpha=float(ema_a))

        # ── Publishers ─────────────────────────────────────────────────────
        self._pub_imu   = self.create_publisher(Imu,             '/imu/data',    10)
        self._pub_left  = self.create_publisher(Range,           '/tof/left',    10)
        self._pub_right = self.create_publisher(Range,           '/tof/right',   10)
        self._pub_diag  = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        # ── Timers (independent rates) ─────────────────────────────────────
        self._imu_timer  = self.create_timer(1.0 / imu_hz,   self._imu_callback)
        self._tof_timer  = self.create_timer(1.0 / tof_hz,   self._tof_callback)
        self._diag_timer = self.create_timer(1.0 / DIAG_RATE_HZ, self._diag_callback)

        self.get_logger().info(
            f"imu_tof_node ready  |  IMU @ {imu_hz:.0f} Hz  |  ToF @ {tof_hz:.0f} Hz"
        )

    # ── Health helpers ────────────────────────────────────────────────────────
    @staticmethod
    def _make_state() -> dict:
        return {'sensor': None, 'healthy': False, 'fails': 0}

    def _mark_ok(self, state: dict) -> None:
        state['healthy'] = True
        state['fails']   = 0

    def _mark_fail(self, state: dict, label: str, error: Exception) -> None:
        state['fails'] += 1
        state['healthy'] = False
        if state['fails'] == 1:
            # Log only on first failure to avoid log spam; /diagnostics carries ongoing state
            self.get_logger().warn(f"[{label}] sensor error: {error}")

    # ── Fault-tolerant sensor init ────────────────────────────────────────────
    def _try_init_imu(self):
        try:
            self._select_channel(MUX_CH_IMU)
            imu = BNO08X_I2C(self._i2c, address=BNO085_ADDR)
            imu.enable_feature(BNO_REPORT_ACCELEROMETER)
            imu.enable_feature(BNO_REPORT_GYROSCOPE)
            imu.enable_feature(BNO_REPORT_ROTATION_VECTOR)
            self._imu_state['healthy'] = True
            self.get_logger().info("BNO085 initialised on CH7")
            return imu
        except Exception as e:
            self.get_logger().warn(f"BNO085 not found on CH7: {e} — will retry")
            return None

    def _try_init_tof(self, channel: int, label: str):
        state = self._tof_l_state if label == 'left' else self._tof_r_state
        try:
            self._select_channel(channel)
            sensor = adafruit_vl53l0x.VL53L0X(self._i2c)
            sensor.measurement_timing_budget = 33000   # µs → ~30 Hz
            state['healthy'] = True
            self.get_logger().info(f"VL53L0X ({label}) initialised on CH{channel}")
            return sensor
        except Exception as e:
            self.get_logger().warn(
                f"VL53L0X ({label}) not found on CH{channel}: {e} — will retry"
            )
            return None

    # ── MUX helpers ──────────────────────────────────────────────────────────
    def _select_channel(self, channel: int) -> None:
        """Activate a single TCA9548A channel (0-7)."""
        while not self._i2c.try_lock():
            pass
        try:
            self._i2c.writeto(self._mux_addr, bytes([1 << channel]))
        finally:
            self._i2c.unlock()

    def _deselect_all(self) -> None:
        """Disable all mux channels (write 0x00)."""
        while not self._i2c.try_lock():
            pass
        try:
            self._i2c.writeto(self._mux_addr, bytes([0x00]))
        finally:
            self._i2c.unlock()

    # ── IMU callback (50 Hz) ─────────────────────────────────────────────────
    def _imu_callback(self) -> None:
        st = self._imu_state

        # Attempt reconnect if sensor is absent or repeatedly failing
        if st['sensor'] is None or st['fails'] >= RECONNECT_AFTER:
            st['sensor'] = self._try_init_imu()
            st['fails']  = 0
            if st['sensor'] is None:
                return

        try:
            self._select_channel(MUX_CH_IMU)

            quat  = self._imu_state['sensor'].quaternion
            accel = self._imu_state['sensor'].linear_acceleration
            gyro  = self._imu_state['sensor'].gyro

            if quat is None or accel is None or gyro is None:
                return

            msg = Imu()
            msg.header.stamp    = self.get_clock().now().to_msg()
            msg.header.frame_id = self._imu_fid

            # Quaternion — BNO08x returns (i, j, k, real); ROS uses (x,y,z,w)
            qi, qj, qk, qr = quat
            msg.orientation.x = qi
            msg.orientation.y = qj
            msg.orientation.z = qk
            msg.orientation.w = qr
            msg.orientation_covariance = [
                0.0025, 0, 0,
                0, 0.0025, 0,
                0, 0, 0.0025,
            ]

            msg.angular_velocity.x = gyro[0]
            msg.angular_velocity.y = gyro[1]
            msg.angular_velocity.z = gyro[2]
            msg.angular_velocity_covariance = [
                0.0001, 0, 0,
                0, 0.0001, 0,
                0, 0, 0.0001,
            ]

            msg.linear_acceleration.x = accel[0]
            msg.linear_acceleration.y = accel[1]
            msg.linear_acceleration.z = accel[2]
            msg.linear_acceleration_covariance = [
                0.01, 0, 0,
                0, 0.01, 0,
                0, 0, 0.01,
            ]

            self._pub_imu.publish(msg)
            self._mark_ok(st)

        except Exception as e:
            self._mark_fail(st, 'IMU', e)

    # ── ToF callback (20 Hz) ─────────────────────────────────────────────────
    def _tof_callback(self) -> None:
        self._read_tof(
            channel  = MUX_CH_TOF_LEFT,
            state    = self._tof_l_state,
            filt     = self._filt_left,
            pub      = self._pub_left,
            frame_id = self._tof_l_fid,
            label    = 'left',
        )
        # Small delay so IR pulses from the two sensors don't overlap
        time.sleep(0.005)
        self._read_tof(
            channel  = MUX_CH_TOF_RIGHT,
            state    = self._tof_r_state,
            filt     = self._filt_right,
            pub      = self._pub_right,
            frame_id = self._tof_r_fid,
            label    = 'right',
        )

    def _read_tof(self, channel, state, filt, pub, frame_id, label) -> None:
        # Attempt reconnect if sensor is absent or repeatedly failing
        if state['sensor'] is None or state['fails'] >= RECONNECT_AFTER:
            state['sensor'] = self._try_init_tof(channel, label)
            state['fails']  = 0
            if state['sensor'] is None:
                return

        try:
            self._select_channel(channel)
            raw_mm = state['sensor'].range          # returns millimetres (int)
            raw_m  = raw_mm / 1000.0

            filtered_m = filt.update(raw_m)
            if filtered_m is None:
                return                              # buffer filling or out-of-range

            msg = Range()
            msg.header.stamp      = self.get_clock().now().to_msg()
            msg.header.frame_id   = frame_id
            msg.radiation_type    = Range.INFRARED
            msg.field_of_view     = VL53_FOV_RAD
            msg.min_range         = VL53_MIN_RANGE
            msg.max_range         = VL53_MAX_RANGE
            msg.range             = filtered_m

            pub.publish(msg)
            self._mark_ok(state)

        except Exception as e:
            self._mark_fail(state, f'ToF-{label}', e)

    # ── Diagnostics callback (1 Hz) ───────────────────────────────────────────
    def _diag_callback(self) -> None:
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [
            self._make_diag('BNO085 IMU',      self._imu_state,   'imu_link'),
            self._make_diag('VL53L0X left',    self._tof_l_state, 'tof_left_link'),
            self._make_diag('VL53L0X right',   self._tof_r_state, 'tof_right_link'),
        ]
        self._pub_diag.publish(arr)

    def _make_diag(self, name: str, state: dict, frame: str) -> DiagnosticStatus:
        s = DiagnosticStatus()
        s.name = f'imu_tof_node: {name}'
        s.hardware_id = frame
        if state['healthy']:
            s.level   = DiagnosticStatus.OK
            s.message = 'OK'
        elif state['sensor'] is None:
            s.level   = DiagnosticStatus.ERROR
            s.message = 'Not found — check wiring or mux channel'
        else:
            s.level   = DiagnosticStatus.WARN
            s.message = f'Unhealthy — {state["fails"]} consecutive failures'
        s.values = [KeyValue(key='consecutive_failures', value=str(state['fails']))]
        return s


# ── Entry point ──────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = ImuTofNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._deselect_all()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()