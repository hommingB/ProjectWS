import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    bringup_pkg = get_package_share_directory('my_robot_bringup')

    slam_config = os.path.join(bringup_pkg, 'config', 'slam_toolbox.yaml')

    # ── 1. Full localization stack (sensors + EKF) ────────────────────────
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'localization.launch.py')
        )
    )

    # ── 2. slam_toolbox — delayed to let EKF and scan come up first ───────
    #       EKF needs 6s internally, add 2s buffer on top = 8s
    slam_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[slam_config],
            )
        ]
    )

    return LaunchDescription([
        localization,   # 1 — all sensors + EKF
        slam_node,      # 2 — SLAM on top
    ])