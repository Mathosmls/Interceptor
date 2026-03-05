from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'nmpc_rover'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='NMPC Controller for ArduPilot Rover via MAVROS',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nmpc_controller = nmpc_rover.nmpc_controller_node:main',
        ],
    },
)
