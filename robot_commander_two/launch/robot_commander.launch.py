from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("robot_commander_two")

    params_file = PathJoinSubstitution([pkg, "config", "waypoints.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=params_file,
            description="Path to the parameter YAML file",
        ),

        Node(
            package="robot_commander_two",
            executable="wp_commander",
            name="waypoint_commander",
            output="screen",
            # remappings=[
            #     # Wire ModeManager outputs → WaypointCommander inputs
            #     ("~/send_goal",    "/mode_manager/send_goal"),
            #     ("~/preempt_goal", "/mode_manager/preempt_goal"),
            # ],
        ),

        Node(
            package="robot_commander_two",
            executable="mode_manager",
            name="mode_manager",
            output="screen",
            parameters=[LaunchConfiguration("params_file")],
        ),
    ])
