from setuptools import find_packages, setup

package_name = 'yolo_coco_cpu'

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
            'yolo_coco_cpu = yolo_coco_cpu.yolo_coco_cpu:main',
        ],
    },
)
