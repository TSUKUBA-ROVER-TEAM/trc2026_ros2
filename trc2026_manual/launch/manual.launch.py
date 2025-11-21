from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='trc2026_manual',
            executable='drive_manual_controller_node',
            output='screen'
        ),
        Node(
            package='joy',
            executable='joy_node',
            output='screen'
        ),
    ])
