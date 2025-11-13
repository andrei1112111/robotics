from setuptools import setup

package_name = 'unicmove'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # если есть launch, rviz и т.д.:
        ('share/' + package_name + '/launch', ['launch/circle_movement.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Circle movement node package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'circle_movement = unicmove.circle_movement:main',
        ],
    },
)
