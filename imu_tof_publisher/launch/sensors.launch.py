"""
launch/sensors.launch.py
Launch the imu_tof_node with tunable parameters.
Usage:
    ros2 launch robot_sensors sensors.launch.py
    ros2 launch robot_sensors sensors.launch.py tof_rate_hz:=15.0 ema_alpha:=0.2
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Declare overridable args ─────────────────────────────────────────────
    args = [
        DeclareLaunchArgument('imu_rate_hz',        default_value='50.0'),
        DeclareLaunchArgument('tof_rate_hz',        default_value='20.0'),
        DeclareLaunchArgument('median_window',      default_value='5'),
        DeclareLaunchArgument('ema_alpha',          default_value='0.3'),
        DeclareLaunchArgument('imu_frame_id',       default_value='imu_link'),
        DeclareLaunchArgument('tof_left_frame_id',  default_value='tof_left_link'),
        DeclareLaunchArgument('tof_right_frame_id', default_value='tof_right_link'),
        DeclareLaunchArgument('tof_toe_in_deg',     default_value='11.3'),
        DeclareLaunchArgument('crossover_dist_m',   default_value='0.40'),
    ]

    node = Node(
        package    = 'imu_tof_publisher',
        executable = 'imu_tof_node',
        name       = 'imu_tof_node',
        output     = 'screen',
        parameters = [{
            'imu_rate_hz':        LaunchConfiguration('imu_rate_hz'),
            'tof_rate_hz':        LaunchConfiguration('tof_rate_hz'),
            'median_window':      LaunchConfiguration('median_window'),
            'ema_alpha':          LaunchConfiguration('ema_alpha'),
            'imu_frame_id':       LaunchConfiguration('imu_frame_id'),
            'tof_left_frame_id':  LaunchConfiguration('tof_left_frame_id'),
            'tof_right_frame_id': LaunchConfiguration('tof_right_frame_id'),
            'tof_toe_in_deg':     LaunchConfiguration('tof_toe_in_deg'),
            'crossover_dist_m':   LaunchConfiguration('crossover_dist_m'),
        }],
    )

    return LaunchDescription(args + [node])
