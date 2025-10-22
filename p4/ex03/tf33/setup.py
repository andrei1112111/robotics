from setuptools import find_packages, setup

package_name = 'tf33'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + '/launch', ['launch/time_travel.launch.py']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',

    entry_points={
        'console_scripts': [
            'turtle1_broadcaster = tf33.turtle1_broadcaster:main',
            'turtle2_broadcaster = tf33.turtle2_broadcaster:main',
            'turtle_time_follower = tf33.turtle_time_follower:main',
        ],
    },
)
