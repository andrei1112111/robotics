import launch
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='unicmove',
            executable='circle_movement',
            name='circle_movement',
            output='screen'
        )
    ])
