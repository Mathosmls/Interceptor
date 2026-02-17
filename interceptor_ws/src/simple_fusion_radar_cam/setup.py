from setuptools import find_packages, setup

package_name = 'simple_fusion_radar_cam'

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
    maintainer='mls',
    maintainer_email='mathis.le_scoezec@ensta.fr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'radar_cam_fusion = simple_fusion_radar_cam.radar_cam_fusion:main',
        ],
    },
)
