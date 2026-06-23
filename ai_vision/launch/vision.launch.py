from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ai_vision',
            executable='vision_node',
            name='ai_vision_node',
            output='screen',
            emulate_tty=True,
        )
    ])
