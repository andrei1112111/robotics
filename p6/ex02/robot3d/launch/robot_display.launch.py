import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():
    # Пакеты
    pkg_robot = get_package_share_directory('robot3d')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Пути до файлов
    urdf_path = os.path.join(pkg_robot, 'urdf', 'robot.xacro')
    rviz_config_path = os.path.join(pkg_robot, 'rviz', 'urdf_config.rviz')
    bridge_config_path = os.path.join(pkg_robot, 'config', 'robot_bridge.yaml')

    # Описание робота через xacro
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

    # Мост ROS<->Gazebo
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config_path}],
        output='screen'
    )

    # Спавн робота в Gazebo через topic robot_description
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'robot',
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '2.0'
        ],
        output='screen',
    )

    static_tf_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'robot/base_link', 'base_link']
    )

    static_tf_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'robot/odom', 'odom']
    )

    # ВИРТУАЛЬНЫЙ 3D LIDAR → POINTCLOUD
    lt3d = Node(
        package='robot3d',
        executable='lidar_to_3d_pointcloud',
        name='lidar_to_3d_pointcloud',
        parameters=[{
            'scan_topic': '/robot/scan',
            'frame_id': 'robot/base_link',
            'output_topic': '/robot/virtual_depth/points',
            'vertical_fov_deg': 30.0,
            'num_layers': 12,
            'max_history_scans': 40,
        }],
        output='screen'
    )

    # POINTCLOUD → DEPTH IMAGE
    lt3img = Node(
        package='robot3d',
        executable='pointcloud_to_depth_image',
        name='pointcloud_to_depth_image',
        output='screen',
        parameters=[{
            "width": 320,
            "height": 240,
            "fx": 200.0,
            "fy": 200.0,
            "cx": 160.0,
            "cy": 120.0,
        }]
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', description='Запуск RViz'),
        gz_sim,
        bridge,
        robot_state_publisher,
        static_tf_base,
        static_tf_odom,
        TimerAction(period=3.0, actions=[spawn_robot]),
        TimerAction(period=5.0, actions=[lt3d, lt3img]),
        rviz
    ])
