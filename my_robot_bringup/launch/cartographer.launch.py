import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ── Paths ────────────────────────────────────────────────────────────────
    bringup_pkg = get_package_share_directory('my_robot_bringup')
    config_dir = os.path.join(bringup_pkg, 'config')

    # ── Launch Arguments ─────────────────────────────────────────────────────
    use_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Whether to start RViz2'
    )

    configuration_basename_arg = DeclareLaunchArgument(
        'configuration_basename',
        default_value='cartographer_2d.lua',
        description='Name of the Cartographer configuration LUA file'
    )

    # ── 1. Cartographer Node ──────────────────────────────────────────────────
    # Starts Cartographer with LUA config, remapping sensor topics to match
    # filtered lidar scan, BNO085 IMU, and fused EKF wheel odometry.
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', LaunchConfiguration('configuration_basename')
        ],
        remappings=[
            ('scan', '/scan_filtered'),
            ('imu', '/imu/data'),
            ('odom', '/odometry/filtered')
        ]
    )

    # ── 2. Occupancy Grid Node ───────────────────────────────────────────────
    # Subscribes to submaps and publishes a standard 2D Occupancy Grid map on /map
    # for Nav2 and path planning.
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        arguments=[
            '-resolution', '0.05',            # Match resolution in meters
            '-publish_period_sec', '1.0'
        ]
    )

    # ── 3. RViz2 (Optional visualization) ────────────────────────────────────
    rviz_config_dir = os.path.join(bringup_pkg, 'rviz', 'slam.rviz')
    
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
        configuration_basename_arg,
        cartographer_node,
        occupancy_grid_node,
        rviz_node
    ])
