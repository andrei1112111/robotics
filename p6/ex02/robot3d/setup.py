from setuptools import setup

package_name = 'robot3d'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_dir={'': '.'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        ('share/' + package_name + '/launch', ['launch/robot_display.launch.py']),
        ('share/' + package_name + '/config', ['config/robot_bridge.yaml']),
        ('share/' + package_name + '/rviz', ['rviz/urdf_config.rviz']),
        ('share/' + package_name + '/urdf', [
            'urdf/robot.xacro',
            'urdf/robot.gazebo.xacro',
            'urdf/robot.sdf'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='test',
    maintainer_email='test@test.com',
    description='Virtual depth from 2D lidar',
    license='Apache-2.0',

    entry_points={
        'console_scripts': [
            'lidar_to_3d_pointcloud = robot3d.lidar_to_3d_pointcloud:main',
            'pointcloud_to_depth_image = robot3d.pointcloud_to_depth_image:main',
        ],
    },
)