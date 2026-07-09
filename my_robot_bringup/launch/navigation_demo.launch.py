import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    bringup_pkg = get_package_share_directory('my_robot_bringup')

    use_map_server = LaunchConfiguration('map_server')

    declare_map_server_cmd = DeclareLaunchArgument(
        'map_server',
        default_value='true',
        description='Whether to start the map server'
    )

    nav2_params    = os.path.join(bringup_pkg, 'config', 'nav2_params_demo.yaml')
    map_file       = os.path.join(bringup_pkg, 'config', 'demo_map1.yaml')
    twist_mux_file = os.path.join(bringup_pkg, 'config', 'twist_mux.yaml')
    bt_xml_file    = os.path.join(bringup_pkg, 'config', 'custom_bt.xml')

    # Fail loudly at launch time if the BT XML wasn't installed,
    # instead of silently leaving bt_navigator inactive at runtime.
    assert os.path.exists(bt_xml_file), (
        f"\n\n[navigation.launch.py] BT XML not found at:\n  {bt_xml_file}\n"
        f"Did you forget to run: colcon build --packages-select my_robot_bringup ?\n"
    )

    # ── 1. Map server — serves the saved map ─────────────────────────────
    map_server = Node(
        condition=IfCondition(use_map_server),
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            nav2_params,
            {'yaml_filename': map_file}
        ]
    )

    # ── 2. AMCL — localizes robot within the saved map ───────────────────
    amcl = Node(
        condition=IfCondition(use_map_server),
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params]
    )

    # ── 3. Keepout / speed filter servers ────────────────────────────────
    keepout_mask_server = Node(
        condition=IfCondition(use_map_server),
        package='nav2_map_server',
        executable='map_server',
        name='keepout_filter_mask_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'yaml_filename': os.path.join(bringup_pkg, 'config', 'demo_map1_keepout.yaml'),
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

    # ── 4. Lifecycle manager for localization stack ───────────────────────
    lifecycle_manager_localization = Node(
        condition=IfCondition(use_map_server),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'bond_timeout': 0.0,
            'node_names': [
                'map_server',
                'amcl',
                'keepout_filter_mask_server',
                'keepout_costmap_filter_info_server',
                'speed_filter_mask_server',
                'speed_costmap_filter_info_server',
            ]
        }]
    )

    # ── 5. Nav2 core nodes ───────────────────────────────────────────────
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
        parameters=[nav2_params],
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
        # Override the BT XML path here so it resolves at launch time
        # via get_package_share_directory — not relying on the yaml value.
        parameters=[nav2_params, {'default_nav_to_pose_bt_xml': bt_xml_file}],
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params],
    )

    # ── 6. Collision monitor ──────────────────────────────────────────────
    # Sits between velocity_smoother (cmd_vel_smoothed) and twist_mux.
    # Slows/stops the robot on proximity via the FootprintApproach + stop
    # polygons defined in nav2_params.yaml.
    # twist_mux gives cmd_vel_safe priority 100 > navigation priority 10,
    # so this filtered signal is always the one reaching the wheels.
    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[nav2_params],
        remappings=[
            ('cmd_vel_in',  'cmd_vel_smoothed'),
            ('cmd_vel_out', 'cmd_vel_safe'),
        ]
    )

    # ── 7. Nav2 lifecycle manager ─────────────────────────────────────────
    # Separated from the nodes it manages so they have time to fully
    # initialize before the manager tries to configure/activate them.
    # Nodes start at t=3s, manager starts at t=6s — 3s gap prevents the
    # "No transition matching 3 found for current state unconfigured" race.
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
                'collision_monitor',
            ]
        }]
    )

    # ── 8. twist_mux ─────────────────────────────────────────────────────
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[twist_mux_file],
        remappings=[
            ('cmd_vel_out', '/diff_drive_controller/cmd_vel')
        ]
    )

    # ── 9. Staggered Nav2 startup ─────────────────────────────────────────
    # t=3s: spawn all Nav2 nodes (they begin their own internal init)
    # t=6s: lifecycle manager starts and finds fully-initialized nodes
    nav2_nodes = TimerAction(
        period=3.0,
        actions=[
            controller_server,
            planner_server,
            behavior_server,
            bt_navigator,
            velocity_smoother,
            collision_monitor,
        ]
    )

    nav2_lifecycle = TimerAction(
        period=6.0,
        actions=[nav2_lifecycle_manager]
    )

    # ── 10. Publish initial pose after AMCL is up ────────────────────────
    initial_pose_pub = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '-1', '/initialpose',
            'geometry_msgs/PoseWithCovarianceStamped',
            '{header: {frame_id: "map"}, pose: {pose: {position: '
            '{x: 17.1414, y: 2.18162, z: 0.0}, orientation: '
            '{x: 0.0, y: 0.0, z: 0.707107, w: 0.707107}}}}'
        ],
        output='screen'
    )
    initial_pose_timer = TimerAction(
        period=10.0,   # pushed to 10s — AMCL needs localization stack + ekf up first
        actions=[initial_pose_pub]
    )

    return LaunchDescription([
        declare_map_server_cmd,
        twist_mux_node,
        map_server,
        amcl,
        keepout_mask_server,
        keepout_filter_info_server,
        speed_mask_server,
        speed_filter_info_server,
        lifecycle_manager_localization,
        nav2_nodes,         # t=3s: nodes start
        nav2_lifecycle,     # t=6s: lifecycle manager activates them
        initial_pose_timer, # t=10s: publish initial pose
    ])