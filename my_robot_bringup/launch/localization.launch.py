import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
        imu_tof_pkg, 'launch', 'sensors.py'
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

    # ── 3. RPLidar filter node ───────────────────────────────────────────────
    rplidar_filter_node = Node(
        package='rplidar_filtered_publisher',
        executable='rplidar_subscriber',
        name='rplidar_filter_node',
        output='screen',
        parameters=[{
            'topic':         '/scan',
            'angle_min_deg': -70.0,
            'angle_max_deg':  70.0,
        }]
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

    return LaunchDescription([
        diff_drive_control,    # 1 — rsp + controllers (has internal timers)
        rplidar_node,          # 2 — /scan
        rplidar_filter_node,   # 3 — /scan_filtered
        i2c_sensors_node,      # 4 — /imu/data, /tof/left, /tof/right
        ekf_node,              # 5 — /odometry/filtered (delayed 6s)
    ])