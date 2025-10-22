from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    delay_arg = DeclareLaunchArgument(
        'delay',
        default_value='2.0',
        description='Задержка (в секундах) для time travel'
    )

    delay = LaunchConfiguration('delay')

    return LaunchDescription([
        delay_arg,

        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),

        Node(
            package='tf33',
            executable='turtle1_broadcaster',
            name='turtle1_broadcaster'
        ),

        ExecuteProcess(
            cmd=[
                'ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn',
                '{x: 4.0, y: 2.0, theta: 0.0, name: "turtle2"}'
            ],
            output='screen'
        ),

        Node(
            package='tf33',
            executable='turtle2_broadcaster',
            name='turtle2_broadcaster'
        ),

        Node(
            package='tf33',
            executable='turtle_time_follower',
            name='turtle_time_follower',
            parameters=[{'delay': delay}],
            output='screen'
        ),
    ]
)
