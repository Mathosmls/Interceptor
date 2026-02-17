from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('ars408_radar')

    cfg_radar_path = os.path.join(pkg_share, 'config', 'radar_cfg.yaml')
    filters_path = os.path.join(pkg_share, 'config', 'filters.yaml')

    return LaunchDescription([
        Node(
            package='ars408_radar',
            executable='radar_node',
            name='radar_node',
            parameters=[
                {"cfg_radar_yaml": cfg_radar_path},
                {"filters_obj_yaml": filters_path}
            ]
        ),

         Node(
            package='ars408_radar',
            executable='radar_objects_rviz',
            name='radar_objects_rviz',
        )
    ])
