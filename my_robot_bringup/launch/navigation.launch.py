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

    # ── 5. Nav2 stack — delayed to let map + AMCL come up first ──────────
    nav2 = TimerAction(
        period=15.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('nav2_bringup'),
                        'launch', 'navigation_launch.py'
                    )
                ),
                launch_arguments={
                    'use_sim_time': 'false',
                    'params_file':  nav2_params,
                }.items()
            )
        ]
    )

    return LaunchDescription([
        localization,                    # 1 — sensors + EKF
        map_server,                      # 2 — serves /map
        amcl,                            # 3 — localizes in map
        lifecycle_manager_localization,  # 4 — activates map_server + amcl
        nav2,                            # 5 — full Nav2 stack (delayed 10s)
    ])