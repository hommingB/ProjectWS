#!/usr/bin/env python3
"""
rtabmap_launch.py — Launches RTAB-Map SLAM fusing:
  • Monocular RGB camera   (/camera/image_raw  + /camera/camera_info)
  • 2-D filtered LiDAR     (/scan_filtered)
  • EKF wheel+IMU odometry (/odometry/filtered)

Sensor roles:
  - LiDAR  → 2-D occupancy grid (metric), scan-matching registration (ICP)
  - Camera → Visual bag-of-words loop-closure detection
  - EKF    → Motion prior for RTAB-Map's pose graph

TF produced:
  map → odom        (by rtabmap)
  odom → base_footprint (by existing ekf_filter_node)

Usage:
  ros2 launch my_robot_bringup rtabmap_launch.py
  ros2 launch my_robot_bringup rtabmap_launch.py viz:=false        # headless
  ros2 launch my_robot_bringup rtabmap_launch.py localization:=true # nav mode
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    bringup_pkg = get_package_share_directory('my_robot_bringup')

    # ── Launch arguments ─────────────────────────────────────────────────────
    viz_arg = DeclareLaunchArgument(
        'viz', default_value='true',
        description='Launch rtabmap_viz (set false on headless Pi 5 runs)',
    )
    localization_arg = DeclareLaunchArgument(
        'localization', default_value='false',
        description='If true, run in localization-only mode against a saved map',
    )

    # ── 1. Camera Info Publisher ─────────────────────────────────────────────
    # Mirrors the timestamp from /camera/image_raw onto a CameraInfo message
    # so that RTAB-Map's ApproximateTime filter can match the two topics.
    # Must be launched before rtabmap so the topic exists at startup.
    camera_info_node = Node(
        package='my_robot_bringup',
        executable='camera_info_publisher',
        name='camera_info_publisher',
        output='screen',
        parameters=[{'frame_id': 'camera_link'}],
    )

    # ── 2. RTAB-Map SLAM node ────────────────────────────────────────────────
    rtabmap_mapping = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('localization')),
        arguments=['--delete_db_on_start'],   # fresh map every run (remove to resume)
        remappings=[
            # ── Inputs ────────────────────────────────────────────────────
            ('rgb/image',       '/camera/image_sync'),
            ('rgb/camera_info', '/camera/info_sync'),
            ('scan',            '/scan_filtered'),
            ('odom',            '/odometry/filtered'),
        ],
        parameters=[{
            # ── Frame IDs ─────────────────────────────────────────────────
            'frame_id':         'base_footprint',   # robot body frame
            'odom_frame_id':    'odom',             # EKF provides odom→base_footprint
            'map_frame_id':     'map',

            # ── Data subscriptions ────────────────────────────────────────
            'subscribe_rgb':    True,
            'subscribe_depth':  False,              # monocular — no depth camera
            'subscribe_scan':   True,               # 2-D LiDAR for grid + ICP
            'subscribe_odom':   True,

            # ── Synchronization ───────────────────────────────────────────
            # approx_sync=true required because image and scan arrive at
            # different rates (30 Hz camera vs 10 Hz LiDAR).
            'approx_sync':              True,
            'approx_sync_max_interval': 2.5,        # increased tolerance to 2.5s for latency offset
            'sync_queue_size':          30,
            'topic_queue_size':         30,

            # ── Occupancy grid from LiDAR (not from depth image) ──────────
            'Grid/FromDepth':   'false',
            'Grid/RayTracing':  'true',
            'Grid/CellSize':    '0.05',

            # ── Registration strategy ─────────────────────────────────────
            # 0=Visual, 1=ICP, 2=Visual+ICP
            # Use ICP (1) because we have no depth — ICP registers LiDAR scans
            'Reg/Strategy':         '1',

            # ── Visual feature matching (loop-closure) ────────────────────
            # Monocular epipolar geometry for feature triangulation
            'Vis/EstimationType':   '2',
            # Minimum visual inliers before accepting a loop closure
            'Vis/MinInliers':       '10',
            # Enable stereo-from-motion so depth can be inferred over time
            'Mem/StereoFromMotion': 'true',
            # Use ORB features — faster than SIFT, good for indoor textures
            'Kp/DetectorStrategy':  '2',            # 2 = ORB

            # ── Memory / graph ────────────────────────────────────────────
            # Minimum time between two keyframes — prevents duplicate nodes
            # when the robot is stationary.
            'Rtabmap/DetectionRate':    '1',        # keyframe attempt rate (Hz)
            'Rtabmap/TimeThr':          '700',      # ms time budget per iteration

            # ── TF publishing ─────────────────────────────────────────────
            'publish_tf':       True,               # rtabmap publishes map→odom

            # ── QoS Settings ──────────────────────────────────────────────
            # 2 = Best Effort (Reliability Policy matching high frequency LiDAR)
            'qos_scan':         2,
        }],
    )

    # Localization-only mode — same params but Mem/IncrementalMemory=false
    rtabmap_localization = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        condition=IfCondition(LaunchConfiguration('localization')),
        remappings=[
            ('rgb/image',       '/camera/image_sync'),
            ('rgb/camera_info', '/camera/info_sync'),
            ('scan',            '/scan_filtered'),
            ('odom',            '/odometry/filtered'),
        ],
        parameters=[{
            'frame_id':         'base_footprint',
            'odom_frame_id':    'odom',
            'map_frame_id':     'map',
            'subscribe_rgb':    True,
            'subscribe_depth':  False,
            'subscribe_scan':   True,
            'subscribe_odom':   True,
            'approx_sync':              True,
            'approx_sync_max_interval': 2.5,
            'sync_queue_size':          30,
            'topic_queue_size':         30,
            'Grid/FromDepth':   'false',
            'Grid/RayTracing':  'true',
            'Grid/CellSize':    '0.05',
            'Reg/Strategy':         '1',
            'Vis/EstimationType':   '2',
            'Vis/MinInliers':       '10',
            'Mem/StereoFromMotion': 'true',
            'Kp/DetectorStrategy':  '2',
            'Rtabmap/DetectionRate':    '1',
            'Rtabmap/TimeThr':          '700',
            'publish_tf':       True,
            # ── Localization only ─────────────────────────────────────────
            'Mem/IncrementalMemory': 'false',       # do NOT grow the map
            'Mem/InitWMWithAllNodes': 'true',       # load entire saved map
            # ── QoS Settings ──────────────────────────────────────────────
            'qos_scan':         2,
        }],
    )

    # ── 3. RTAB-Map Visualizer (optional) ────────────────────────────────────
    # Provides a rich 3-D viewer: point cloud, loop closure graph, odometry path.
    # Disable on Raspberry Pi 5 (headless) by passing viz:=false.
    rtabmap_viz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        condition=IfCondition(LaunchConfiguration('viz')),
        remappings=[
            ('rgb/image',       '/camera/image_sync'),
            ('rgb/camera_info', '/camera/info_sync'),
            ('scan',            '/scan_filtered'),
            ('odom',            '/odometry/filtered'),
        ],
        parameters=[{
            'frame_id':        'base_footprint',
            'odom_frame_id':   'odom',
            'subscribe_rgb':   True,
            'subscribe_depth': False,
            'subscribe_scan':  True,
            'approx_sync':     True,
            'approx_sync_max_interval': 2.5,
            'sync_queue_size': 30,
            'topic_queue_size': 30,
            # ── QoS Settings ──────────────────────────────────────────────
            'qos_scan':         2,
        }],
    )

    return LaunchDescription([
        viz_arg,
        localization_arg,
        #camera_info_node,
        rtabmap_mapping,
        rtabmap_localization,
        rtabmap_viz_node,
    ])
