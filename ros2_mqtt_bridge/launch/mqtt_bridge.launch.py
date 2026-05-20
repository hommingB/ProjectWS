from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_mqtt_bridge',
            executable='mqtt_bridge_node',
            name='ros2_mqtt_bridge',
            output='screen',
            parameters=[
                # MQTT Broker Connection (local broker with bridge to main)
                # The local mosquitto broker should be running on localhost:1883
                # Use the broker_setup.sh script to configure the local broker
                {'mqtt_host': 'localhost'},
                {'mqtt_port': 1883},
            ]
        )
    ])
