from setuptools import setup
from glob import glob


package_name = 'action_cleaning_robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    zip_safe=True,
    author='a',
    author_email='a@a.com',
    description='ROS2 package for cleaning turtle via Actions',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'cleaning_action_server = action_cleaning_robot.cleaning_action_server:main',
            'cleaning_action_client = action_cleaning_robot.cleaning_action_client:main',
        ],
    },
)