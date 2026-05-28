import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    bringup_pkg = get_package_share_directory('my_robot_bringup')
    slam_config = os.path.join(bringup_pkg, 'config', 'slam_toolbox_pc.yaml')

    # Optional RViz launch argument
    use_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Whether to start RViz2'
    )

    # ── 1. slam_toolbox (PC-only) ──────────────────────────────────────────
    # Starts immediately on the PC
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config],
    )

    # Configure slam_toolbox after it starts (2s delay)
    configure_slam = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'configure'],
                output='screen'
            )
        ]
    )

    # Activate slam_toolbox after configure (4s delay)
    activate_slam = TimerAction(
        period=4.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'activate'],
                output='screen'
            )
        ]
    )

    # ── 2. RViz2 (PC-only) ──────────────────────────────────────────────────
    # Optional: Launches RViz with a default configuration if 'rviz:=true'
    rviz_config_dir = os.path.join(bringup_pkg, 'rviz', 'slam.rviz')
    
    # Fallback to general RViz if custom config doesn't exist
    if os.path.exists(rviz_config_dir):
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen'
        )
    else:
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen'
        )

    return LaunchDescription([
        use_rviz_arg,
        slam_node,
        configure_slam,
        activate_slam,
        rviz_node
    ])
