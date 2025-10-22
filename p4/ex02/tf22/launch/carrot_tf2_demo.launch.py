from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    radius = LaunchConfiguration('radius', default='1.0')
    direction = LaunchConfiguration('direction_of_rotation', default='1')

    return LaunchDescription([
        DeclareLaunchArgument('radius', default_value='1.0'),
        DeclareLaunchArgument('direction_of_rotation', default_value='1'),

        Node(package='turtlesim', executable='turtlesim_node', name='sim'),

        # Node(package='turtlesim', executable='turtle_teleop_key', name='teleop', output='screen'),

        ExecuteProcess(
            cmd=[
                'ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn',
                '{x: 4.0, y: 2.0, theta: 0.0, name: "turtle2"}'
            ],
            output='screen'
        ),


        Node(package='tf22', executable='turtle_broadcaster', name='turtle1_broadcaster',
             parameters=[{'turtle': 'turtle1'}]),

        Node(package='tf22', executable='turtle_broadcaster', name='turtle2_broadcaster',
             parameters=[{'turtle': 'turtle2'}]),

        Node(package='tf22', executable='carrot_broadcaster', name='carrot_broadcaster',
             parameters=[{'radius': radius, 'direction_of_rotation': direction}]),

        Node(package='tf22', executable='turtle_listener', name='turtle_listener'),
    ])
