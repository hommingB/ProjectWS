import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    bringup_pkg = get_package_share_directory('my_robot_bringup')

    nav2_params  = os.path.join(bringup_pkg, 'config', 'nav2_params.yaml')
    map_file     = os.path.join(bringup_pkg, 'config', 'my_map.yaml')

    # ── 1. Full localization stack (sensors + EKF) ────────────────────────
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'localization.launch.py')
        )
    )

    # ── 2. Map server — serves the saved map ─────────────────────────────
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            nav2_params,
            {'yaml_filename': map_file}   # override the empty string in yaml
        ]
    )

    # ── 3. AMCL — localizes robot within the saved map ───────────────────
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params]
    )

    # ── 4. Nav2 lifecycle manager for map_server + amcl ──────────────────
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server', 'amcl']
        }]
    )

    # ── 5. Nav2 nodes — defined first ────────────────────────────────────
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params],
        remappings=[('cmd_vel', '/diff_drive_controller/cmd_vel')]
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params]
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params],
        remappings=[('cmd_vel', '/diff_drive_controller/cmd_vel')]
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params]
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params],
        remappings=[
            ('cmd_vel', '/diff_drive_controller/cmd_vel'),
            ('cmd_vel_smoothed', '/diff_drive_controller/cmd_vel')
        ]
    )

    nav2_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'bond_timeout': 0.0,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'velocity_smoother',
            ]
        }]
    )
    static_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    # ── 6. Wrap them all in a TimerAction ────────────────────────────────
    nav2 = TimerAction(
        period=8.0,
        actions=[
            controller_server,
            planner_server,
            behavior_server,
            bt_navigator,
            velocity_smoother,
            nav2_lifecycle_manager,
        ]
    )

    return LaunchDescription([
        # static_map_odom,
        localization,
        map_server,
        amcl,
        lifecycle_manager_localization,
        nav2,
    ])