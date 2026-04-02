"""
service_robot_launch.py
-----------------------
Launches:
  1. tof_publisher_node      — reads VL53L0X sensors, publishes /tof/*
  2. tof_safety_node         — intercepts cmd_vel for obstacle safety
  3. tof_to_scan_bridge_node — virtual LaserScan for costmap
  4. person_detection_fusion_node — camera + lidar person tracking

Usage:
  ros2 launch service_robot service_robot_launch.py

  Optionally override params:
    ros2 launch service_robot service_robot_launch.py \
      stop_dist_m:=0.25 \
      slow_dist_m:=0.50 \
      publish_debug_img:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg = FindPackageShare('service_robot')
    cfg = PathJoinSubstitution([pkg, 'config', 'tof_params.yaml'])

    # ── Launch arguments (override on CLI) ────────────────────────
    args = [
        DeclareLaunchArgument('stop_dist_m',          default_value='0.20'),
        DeclareLaunchArgument('slow_dist_m',          default_value='0.45'),
        DeclareLaunchArgument('publish_rate_hz',      default_value='20.0'),
        DeclareLaunchArgument('publish_debug_img',    default_value='true'),
        DeclareLaunchArgument('camera_hfov_deg',      default_value='60.0'),
        DeclareLaunchArgument('approach_dist_m',      default_value='0.80'),
        DeclareLaunchArgument('i2c_bus',              default_value='0'),
        DeclareLaunchArgument('mux_address',          default_value='0x70'),
        DeclareLaunchArgument('left_mux_channel',     default_value='0'),
        DeclareLaunchArgument('right_mux_channel',    default_value='1'),
        DeclareLaunchArgument('use_hog',              default_value='true'),
    ]

    # ── 1. ToF publisher ──────────────────────────────────────────
    tof_node = Node(
        package='service_robot',
        executable='tof_publisher_node',
        name='tof_publisher_node',
        parameters=[cfg, {
            'publish_rate_hz':   LaunchConfiguration('publish_rate_hz'),
            'i2c_bus':           LaunchConfiguration('i2c_bus'),
            'mux_address':       LaunchConfiguration('mux_address'),
            'left_mux_channel':  LaunchConfiguration('left_mux_channel'),
            'right_mux_channel': LaunchConfiguration('right_mux_channel'),
        }],
        output='screen',
        emulate_tty=True,
    )

    # ── 2. Safety node ────────────────────────────────────────────
    safety_node = Node(
        package='service_robot',
        executable='tof_safety_node',
        name='tof_safety_node',
        parameters=[{
            'stop_dist_m':  LaunchConfiguration('stop_dist_m'),
            'slow_dist_m':  LaunchConfiguration('slow_dist_m'),
            'input_topic':  '/cmd_vel_nav',
            'output_topic': '/cmd_vel',
        }],
        output='screen',
        emulate_tty=True,
    )

    # ── 3. Virtual scan bridge ────────────────────────────────────
    scan_bridge_node = Node(
        package='service_robot',
        executable='tof_to_scan_bridge_node',
        name='tof_to_scan_bridge_node',
        parameters=[{
            'left_sensor_bearing_deg':  55.0,
            'right_sensor_bearing_deg': -55.0,
            'scan_frame_id':            'base_link',
        }],
        output='screen',
    )

    # ── 4. Person detection + camera-lidar fusion ─────────────────
    person_node = Node(
        package='service_robot',
        executable='person_detection_fusion_node',
        name='person_detection_fusion_node',
        parameters=[{
            'use_hog':           LaunchConfiguration('use_hog'),
            'publish_debug_img': LaunchConfiguration('publish_debug_img'),
            'camera_hfov_deg':   LaunchConfiguration('camera_hfov_deg'),
            'approach_dist_m':   LaunchConfiguration('approach_dist_m'),
        }],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription(args + [
        tof_node,
        safety_node,
        scan_bridge_node,
        person_node,
    ])
