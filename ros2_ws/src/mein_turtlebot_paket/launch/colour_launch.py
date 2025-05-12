from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mein_turtlebot_paket',
            executable='colour_detection',
            name='colour_detection',
            output='screen',
        )
    ])