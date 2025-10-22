from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    switch_threshold = LaunchConfiguration('switch_threshold', default='1.0')

    return LaunchDescription([
        DeclareLaunchArgument('switch_threshold', default_value='1.0'),

        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),

        # Спавним три черепахи
        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn',
                 '{x: 4.0, y: 2.0, theta: 0.0, name: "turtle2"}'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn',
                 '{x: 8.0, y: 6.0, theta: 0.0, name: "turtle3"}'],
            output='screen'
        ),

        # TF broadcasters для каждой черепашки
        Node(package='tf2b', executable='turtle_broadcaster', name='turtle1_broadcaster',
             parameters=[{'turtle': 'turtle1'}]),
        Node(package='tf2b', executable='turtle_broadcaster', name='turtle2_broadcaster',
             parameters=[{'turtle': 'turtle2'}]),
        Node(package='tf2b', executable='turtle_broadcaster', name='turtle3_broadcaster',
             parameters=[{'turtle': 'turtle3'}]),

        # Узел публикующий TF для carrot1 carrot2 и static_target
        Node(
            package='tf2b',
            executable='target_switcher',
            name='target_switcher',
            parameters=[{'switch_threshold': switch_threshold}]
        ),

        # Контроллер для turtle2 двигающейся за активной целью
        Node(
            package='tf2b',
            executable='turtle_controller',
            name='turtle_controller'
        ),
    ])
