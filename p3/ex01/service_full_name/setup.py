from setuptools import setup, find_packages

package_name = 'service_full_name'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='',
    maintainer_email='',
    description='Full Name Concat',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'service_name = service_full_name.service_name:main',
            'client_name = service_full_name.client_name:main',
        ],
    },
)