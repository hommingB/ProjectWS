import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ── Paths ────────────────────────────────────────────────────────────────
    bringup_pkg      = get_package_share_directory('my_robot_bringup')
    diff_drive_pkg   = get_package_share_directory('robot_diffdrive_controller')
    imu_tof_pkg      = get_package_share_directory("imu_tof_publisher")
    rplidar_launch   = os.path.join(bringup_pkg, 'launch', 'rplidar.launch.py')
    ekf_config       = os.path.join(bringup_pkg, 'config', 'ekf.yaml')

    diff_drive_launch = os.path.join(
        diff_drive_pkg, 'launch', 'diff_drive.launch.py'
    )
    i2c_sensors_launch = os.path.join(
        imu_tof_pkg, 'launch', 'sensors.launch.py'
    )

    # ── Launch Arguments ─────────────────────────────────────────────────────
    use_cam_arg = DeclareLaunchArgument(
        'use_cam',
        default_value='false',
        description='If true, start the Logitech C270 camera driver and camera_info_publisher'
    )

    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video0',
        description='Path to the V4L2 video device for the camera'
    )

    # ── 1. ros2_control ──
    #       Already includes: robot_state_publisher, controller_manager,
    #                         joint_state_broadcaster, diff_drive_controller
    diff_drive_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(diff_drive_launch),
    )

    # ── 2. RPLidar driver ────────────────────────────────────────────────────
    rplidar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rplidar_launch),
        launch_arguments={
            'frame_id': 'lidar_link', 
        }.items()
    )

    #── 3. RPLidar filter node (commented out) ─────────────────────────────────
    rplidar_filter_node = Node(
        package='rplidar_filtered_publisher',
        executable='rplidar_subscriber',
        name='rplidar_filter_node',
        output='screen',
        parameters=[{
            'topic':         '/scan',
            'angle_min_deg': 120.0,
            'angle_max_deg':-113.0,
        }]
    )

    # ── 3b. Laser filters node ────────────────────────────────────────────────
    # laser_filters_node = Node(
    #     package='laser_filters',
    #     executable='scan_to_scan_filter_chain',
    #     name='laser_filters',
    #     output='screen',
    #     parameters=[os.path.join(bringup_pkg, 'config', 'lidar_filters.yaml')],
    # )

    # ── 3a. BNO085 only node ───────────────────────────────────────────────
    bno085_node = Node(
        package='bno085_publisher_py',
        executable='bno085_node',
        name='bno085_node',
        output='screen',
    )

    # ── 4. BNO085 IMU & ToFs node ───────────────────────────────────────────────────
    i2c_sensors_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(i2c_sensors_launch),
    )

    # ── 5. EKF — delayed to let odom + IMU come up first ────────────────────
    #       diff_drive.launch.py already has 2s + 3s timers internally,
    #       so we wait 6s to be safe before starting EKF
    ekf_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[ekf_config],
            )
        ]
    )

    # ── Camera Driver Node ────────────────────────────────────────────────────
    camera_node = Node(
        package='image_tools',
        executable='cam2image',
        name='camera_fusion',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_cam')),
        parameters=[{'device': LaunchConfiguration('video_device')}],
        arguments=['--ros-args', '--log-level', 'warn'],
        remappings=[
            ('image', '/camera/image_raw'),
        ],
    )

    # ── Camera Info Node ──────────────────────────────────────────────────────
    camera_info_node = Node(
        package='my_robot_bringup',
        executable='camera_info_publisher',
        name='camera_info_publisher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_cam')),
        parameters=[{'frame_id': 'camera_link'}],
    )

    return LaunchDescription([
        use_cam_arg,
        video_device_arg,
        diff_drive_control,    # 1 — rsp + controllers (has internal timers)
        rplidar_node,          # 2 — /scan
        rplidar_filter_node,   # 3 — /scan_filtered (disabled)
        # laser_filters_node,    # 3b — laser filter chain
        bno085_node,
        # i2c_sensors_node,      # 4 — /imu/data, /tof/left, /tof/right
        ekf_node,              # 5 — /odometry/filtered (delayed 6s)
        camera_node,
        camera_info_node,
    ])