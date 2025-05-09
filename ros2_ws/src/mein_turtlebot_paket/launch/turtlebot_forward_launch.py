#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Launch-Funktion für den Turtlebot Forward Node."""
    return LaunchDescription([
        Node(
            package='mein_turtlebot_paket',
            executable='turtlebot_forward_node',
            name='turtlebot_forward_node',
            output='screen',
            emulate_tty=True,
        )
    ])