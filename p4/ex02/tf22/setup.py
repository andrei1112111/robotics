from setuptools import setup

package_name = 'tf22'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/carrot_tf2_demo.launch.py']),
        ('share/' + package_name + '/rviz', ['rviz/carrot.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='i',
    maintainer_email='i@i.todo',
    description='TF2 demo with carrot frame',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'turtle_broadcaster = tf22.turtle_broadcaster:main',
            'turtle_listener = tf22.turtle_listener:main',
            'carrot_broadcaster = tf22.carrot_broadcaster:main',
        ],
    },
)
