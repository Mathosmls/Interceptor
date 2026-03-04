import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('visual_servowing'),
        'config',
        'visual_control.yaml'
    )

    control_node = Node(
        package='visual_servowing',
        executable='control_node_exec',
        name='control_node',
        output='screen',
        parameters=[params_file]
    )

    return LaunchDescription([control_node])
