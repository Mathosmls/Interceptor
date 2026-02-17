from setuptools import find_packages, setup
import os 
from glob import glob
package_name = 'ars408_radar'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['ars408_radar', 'ars408_radar.*', 'radar_lib', 'radar_lib.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
        glob('launch/*.py')),  
    ],
    install_requires=['setuptools','ars408_interfaces'],
    zip_safe=True,
    maintainer='mls',
    maintainer_email='mathis.le_scoezec@ensta.fr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'radar_node = ars408_radar.ars408_radar_node:main',
            'radar_objects_rviz = ars408_radar.ars408_rviz:main',
            
        ],
    },
)
