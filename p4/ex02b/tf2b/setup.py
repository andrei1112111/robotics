from setuptools import setup
import os
from glob import glob

package_name = 'tf2b'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='i',
    maintainer_email='i@i.i',
    description='Multi-target TF2 example with turtlesim',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'target_switcher = tf2b.target_switcher:main',
            'turtle_controller = tf2b.turtle_controller:main',
            'turtle_broadcaster = tf2b.turtle_broadcaster:main'
        ],
    },
)
