"""
Launch file — ESP32 diff drive via ros2_control hardware plugin
===============================================================
No micro-ROS agent needed. The C++ plugin owns the serial port.

Starts:
  1. robot_state_publisher  (URDF → TF)
  2. controller_manager     (loads our plugin from URDF, runs control loop)
  3. joint_state_broadcaster spawner
  4. diff_drive_controller  spawner

Usage:
  ros2 launch esp32_hardware diff_drive.launch.py
  ros2 launch esp32_hardware diff_drive.launch.py serial_port:=/dev/ttyACM0
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # ── Paths ──────────────────────────────────────────────────────────────
    pkg_share   = FindPackageShare("robot_diffdrive_controller")
    urdf_file   = PathJoinSubstitution([FindPackageShare("my_robot_description"), "my_robot", "urdf", "robot.urdf.xacro"])
    config_file = PathJoinSubstitution([pkg_share, "config", "diff_drive_controller.yaml"])

    # Inject serial_port argument into xacro so URDF <param> gets the right value.
    # xacro reads it as a command-line arg:  xacro robot.urdf.xacro serial_port:=/dev/ttyACM0
    robot_description_content = Command([
        FindExecutable(name="xacro"), " ",
        urdf_file, " ",
    ])
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # ── 1. Robot state publisher ───────────────────────────────────────────
    robot_state_publisher = Node(
        package    = "robot_state_publisher",
        executable = "robot_state_publisher",
        output     = "screen",
        parameters = [robot_description],
    )

    # ── 2. Controller manager (loads esp32_hardware plugin + runs loop) ────
    #       Reads joint state via plugin read(), sends commands via write().
    controller_manager = Node(
        package    = "controller_manager",
        executable = "ros2_control_node",
        output     = "screen",
        parameters = [robot_description, config_file],
    )

    # ── 3. joint_state_broadcaster (publishes /joint_states from hw state) ─
    joint_state_broadcaster_spawner = TimerAction(
        period = 2.0,
        actions = [Node(
            package    = "controller_manager",
            executable = "spawner",
            arguments  = ["joint_state_broadcaster",
                          "--controller-manager", "/controller_manager"],
            output     = "screen",
        )],
    )

    # ── 4. diff_drive_controller ───────────────────────────────────────────
    diff_drive_spawner = TimerAction(
        period = 3.0,
        actions = [Node(
            package    = "controller_manager",
            executable = "spawner",
            arguments  = ["diff_drive_controller",
                          "--controller-manager", "/controller_manager"],
            output     = "screen",
        )],
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        diff_drive_spawner,
    ])
