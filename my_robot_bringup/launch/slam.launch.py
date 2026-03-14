import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess

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
    
    # Configure slam_toolbox after it starts
    configure_slam = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'configure'],
                output='screen'
            )
        ]
    )

    # Activate slam_toolbox after configure
    activate_slam = TimerAction(
        period=12.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'activate'],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        localization,
        slam_node,        # starts at 8s
        configure_slam,   # configures at 10s
        activate_slam,    # activates at 12s
    ])