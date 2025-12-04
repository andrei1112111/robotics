from setuptools import setup

package_name = 'robot3l'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        ('share/' + package_name, ['package.xml']),

        ('share/' + package_name + '/launch', [
            'launch/robot_display.launch.py'
        ]),

        ('share/' + package_name + '/urdf', [
            'urdf/robot.xacro',
            'urdf/robot_imu.gazebo.xacro',
            'urdf/robot_imu.gazebo',
        ]),

        ('share/' + package_name + '/worlds', [
            'worlds/imu_world.sdf'
        ]),

        ('share/' + package_name + '/rviz', [
            'rviz/urdf_config.rviz'
        ]),

        ('share/' + package_name + '/config', [
            'config/robot_bridge.yaml'
        ]),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Example ROS2 package ex01',
    license='Apache License 2.0',
    tests_require=['pytest'],

    entry_points={
        'console_scripts': [],
    },
)
