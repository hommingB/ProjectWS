#!/usr/bin/env python3
"""
BNO085 IMU Publisher Node for ROS2
-----------------------------------
Reads IMU data from the BNO085 over I2C and publishes:
  - /imu/data       → sensor_msgs/Imu  (quaternion + angular velocity + linear accel)
  - /imu/mag        → sensor_msgs/MagneticField
  - /imu/temp       → sensor_msgs/Temperature

Hardware: BNO085 connected to Raspberry Pi 5 via I2C
  SDA → GPIO2 (Pin 3)
  SCL → GPIO3 (Pin 5)
  VIN → 3.3V  (Pin 1)
  GND → GND   (Pin 6)
  PS0/PS1 → GND for I2C mode

Requires:  pip3 install adafruit-circuitpython-bno08x
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Imu, MagneticField, Temperature
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Header

import board
import busio
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_LINEAR_ACCELERATION,   # ← add this
)

from adafruit_bno08x.i2c import BNO08X_I2C

import math


class BNO085Node(Node):
    def __init__(self):
        super().__init__("bno085_node")

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("i2c_address", 0x4B)          # default; 0x4B if ADR pin HIGH
        self.declare_parameter("publish_tf", False)
        self.declare_parameter("parent_frame_id", "base_link")

        self.frame_id       = self.get_parameter("frame_id").value
        rate_hz             = self.get_parameter("publish_rate_hz").value
        self.i2c_address    = self.get_parameter("i2c_address").value
        self.publish_tf     = self.get_parameter("publish_tf").value
        self.parent_frame   = self.get_parameter("parent_frame_id").value

        # ── QoS ─────────────────────────────────────────────────────────────
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── Publishers ───────────────────────────────────────────────────────
        self.imu_pub  = self.create_publisher(Imu,           "/imu/data", qos)
        self.mag_pub  = self.create_publisher(MagneticField, "/imu/mag",  qos)
        self.temp_pub = self.create_publisher(Temperature,   "/imu/temp", qos)

        # # ── TF broadcaster ───────────────────────────────────────────────────
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        # ── BNO085 init ──────────────────────────────────────────────────────
        self.get_logger().info(
            f"Connecting to BNO085 at I2C address 0x{self.i2c_address:02X} …"
        )
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.bno = BNO08X_I2C(i2c, address=self.i2c_address)
            self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
            self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
            self.bno.enable_feature(BNO_REPORT_MAGNETOMETER)
            self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
            self.bno.enable_feature(BNO_REPORT_LINEAR_ACCELERATION)   # ← add this
            self.get_logger().info("BNO085 initialised successfully ✓")
        except Exception as e:
            import traceback
            self.get_logger().fatal(f"Failed to init BNO085: {e}")
            self.get_logger().fatal(traceback.format_exc())
            raise SystemExit(1)

        # ── Timer ────────────────────────────────────────────────────────────
        period = 1.0 / rate_hz
        self.timer = self.create_timer(period, self.publish_callback)
        self.get_logger().info(f"Publishing at {rate_hz} Hz on /imu/data, /imu/mag, /imu/temp")

    # ────────────────────────────────────────────────────────────────────────
    def publish_callback(self):
        now = self.get_clock().now().to_msg()

        try:
            quat_i, quat_j, quat_k, quat_real = self.bno.quaternion
            accel_x, accel_y, accel_z         = self.bno.acceleration
            gyro_x,  gyro_y,  gyro_z          = self.bno.gyro
            mag_x,   mag_y,   mag_z           = self.bno.magnetic
        except Exception as e:
            self.get_logger().warn(f"Read error: {e}", throttle_duration_sec=2.0)
            return

        # ── IMU message ──────────────────────────────────────────────────────
        imu_msg = Imu()
        imu_msg.header = Header(frame_id=self.frame_id, stamp=now)

        # Orientation – ROS uses (x, y, z, w); BNO085 gives (i, j, k, real)
        imu_msg.orientation.x = float(quat_i)
        imu_msg.orientation.y = float(quat_j)
        imu_msg.orientation.z = float(quat_k)
        imu_msg.orientation.w = float(quat_real)

        # Angular velocity (rad/s)
        imu_msg.angular_velocity.x = float(gyro_x)
        imu_msg.angular_velocity.y = float(gyro_y)
        imu_msg.angular_velocity.z = float(gyro_z)

        # Linear acceleration (m/s²) – gravity removed by BNO085
        imu_msg.linear_acceleration.x = float(accel_x)
        imu_msg.linear_acceleration.y = float(accel_y)
        imu_msg.linear_acceleration.z = float(accel_z)

        # Covariance – set to -1 if unknown; tune with real noise specs
        imu_msg.orientation_covariance[0]         = 0.0025
        imu_msg.orientation_covariance[4]         = 0.0025
        imu_msg.orientation_covariance[8]         = 0.0025
        imu_msg.angular_velocity_covariance[0]    = 0.0001
        imu_msg.angular_velocity_covariance[4]    = 0.0001
        imu_msg.angular_velocity_covariance[8]    = 0.0001
        imu_msg.linear_acceleration_covariance[0] = 0.01
        imu_msg.linear_acceleration_covariance[4] = 0.01
        imu_msg.linear_acceleration_covariance[8] = 0.01

        self.imu_pub.publish(imu_msg)

        # ── Magnetometer message ─────────────────────────────────────────────
        mag_msg = MagneticField()
        mag_msg.header = Header(frame_id=self.frame_id, stamp=now)
        mag_msg.magnetic_field.x = float(mag_x) * 1e-6   # µT → T
        mag_msg.magnetic_field.y = float(mag_y) * 1e-6
        mag_msg.magnetic_field.z = float(mag_z) * 1e-6
        self.mag_pub.publish(mag_msg)

        # ── Temperature message ──────────────────────────────────────────────
        temp_msg = Temperature()
        temp_msg.header   = Header(frame_id=self.frame_id, stamp=now)
        temp_msg.temperature = float(self.bno.temperature) if hasattr(self.bno, "temperature") else 0.0
        self.temp_pub.publish(temp_msg)

        # ── TF transform ─────────────────────────────────────────────────────
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp    = now
            t.header.frame_id = self.parent_frame
            t.child_frame_id  = self.frame_id
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            t.transform.rotation.x = float(quat_i)
            t.transform.rotation.y = float(quat_j)
            t.transform.rotation.z = float(quat_k)
            t.transform.rotation.w = float(quat_real)
            self.tf_broadcaster.sendTransform(t)


# ────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = BNO085Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
