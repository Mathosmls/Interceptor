from setuptools import find_packages, setup
import os 
from glob import glob
package_name = 'gz2ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
        glob('launch/*.py')),  
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mls',
    maintainer_email='mathis.le_scoezec@ensta.fr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'tf_pub_node = gz2ros.tf_pub_node:main',
        'tf_pub_sensors = gz2ros.tf_pub_sensors:main',
        'process_lidar_node = gz2ros.process_lidar:main',
        'radar_object_speed = gz2ros.radar_object_speed:main',
        ],
    },
)
