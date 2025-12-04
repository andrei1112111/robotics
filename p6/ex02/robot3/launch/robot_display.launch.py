import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():
    # Пути до нужных пакетов
    pkg_robot = get_package_share_directory('robot3')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Пути до файлов URDF и RViz
    urdf_path = os.path.join(pkg_robot, 'urdf', 'robot.xacro')
    rviz_config_path = os.path.join(pkg_robot, 'rviz', 'urdf_config.rviz')

    # Описание робота через Xacro
    robot_desc = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    # Запуск Gazebo с пустым миром
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # Публикация состояния звеньев (TF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # Мост между ROS и Gazebo (bridge)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_robot, 'config', 'robot_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    # --- Добавляем мост для joint_states ---
    joint_states_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/model/robot/joint_state@sensor_msgs/msg/JointState@ros2'],
        output='screen'
    )

    # Спавн робота в Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'robot',
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.1'
        ],
        output='screen',
    )

    # RViz визуализация
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen'
    )

    # Возврат описания запуска
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', description='Запуск RViz'),
        gz_sim,
        bridge,
        joint_states_bridge, 
        robot_state_publisher,
        TimerAction(period=5.0, actions=[spawn_robot]),
        rviz
    ])
