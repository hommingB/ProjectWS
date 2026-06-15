import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    bringup_pkg = get_package_share_directory('my_robot_bringup')

    use_map_server = LaunchConfiguration('map_server')

    declare_map_server_cmd = DeclareLaunchArgument(
        'map_server',
        default_value='true',
        description='Whether to start the map server'
    )

    nav2_params  = os.path.join(bringup_pkg, 'config', 'nav2_params.yaml')
    map_file     = os.path.join(bringup_pkg, 'config', 'my_vietduc_3b_map.yaml')
    twist_mux_file = os.path.join(bringup_pkg, 'config', 'twist_mux.yaml')

    # ── 1. Full localization stack (sensors + EKF) ────────────────────────
    # localization = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(bringup_pkg, 'launch', 'localization.launch.py')
    #     )
    # )

    # ── 2. Map server — serves the saved map ─────────────────────────────
    map_server = Node(
        condition=IfCondition(use_map_server),
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
        condition=IfCondition(use_map_server),
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params]
    )

    # ── 4. Nav2 lifecycle manager for map_server + amcl + filters ────────
    keepout_mask_server = Node(
        condition=IfCondition(use_map_server),
        package='nav2_map_server',
        executable='map_server',
        name='keepout_filter_mask_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'yaml_filename': os.path.join(bringup_pkg, 'config', 'my_vietduc_3b_map_keepout.yaml'),
        }],
        remappings=[('map', '/keepout_filter_mask')]
    )

    keepout_filter_info_server = Node(
        condition=IfCondition(use_map_server),
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='keepout_costmap_filter_info_server',
        output='screen',
        parameters=[nav2_params]
    )

    speed_mask_server = Node(
        condition=IfCondition(use_map_server),
        package='nav2_map_server',
        executable='map_server',
        name='speed_filter_mask_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'yaml_filename': os.path.join(bringup_pkg, 'config', 'my_vietduc_3b_map_speed.yaml'),
        }],
        remappings=[('map', '/speed_filter_mask')]
    )

    speed_filter_info_server = Node(
        condition=IfCondition(use_map_server),
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='speed_costmap_filter_info_server',
        output='screen',
        parameters=[nav2_params]
    )

    lifecycle_manager_localization = Node(
        condition=IfCondition(use_map_server),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': [
                'map_server',
                'amcl',
                'keepout_filter_mask_server',
                'keepout_costmap_filter_info_server',
                'speed_filter_mask_server',
                'speed_costmap_filter_info_server'
            ]
        }]
    )

    # ── 5. Nav2 nodes — defined first ────────────────────────────────────
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params],
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
        remappings=[]
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
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[twist_mux_file],
        remappings=[
            ('cmd_vel_out', '/diff_drive_controller/cmd_vel')
        ]
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

    # ── 7. Publish initial pose after Nav2 is up ───────────────────────
    initial_pose_pub = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '-1', '/initialpose',
            'geometry_msgs/PoseWithCovarianceStamped',
            '{header: {frame_id: "map"}, pose: {pose: {position: {x: 6.62135, y: 6.5234, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'
        ],
        output='screen'
    )
    initial_pose_timer = TimerAction(
        period=5.0,
        actions=[initial_pose_pub]
    )

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
        declare_map_server_cmd,
        twist_mux_node,
        # localization,
        map_server,
        amcl,
        keepout_mask_server,
        keepout_filter_info_server,
        speed_mask_server,
        speed_filter_info_server,
        lifecycle_manager_localization,
        nav2,
    ])