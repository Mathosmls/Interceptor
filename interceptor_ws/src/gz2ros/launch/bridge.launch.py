from pathlib import Path
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch_ros.actions import Node

def generate_launch_description():


    # =============================
    # Launch configurations (URDF)
    # =============================
    urdf_interceptor = LaunchConfiguration('urdf_interceptor')
    urdf_target = LaunchConfiguration('urdf_target')

    declare_urdf_interceptor = DeclareLaunchArgument(
        'urdf_interceptor',
        default_value=str(
            Path(get_package_share_directory('gz_sim_one')) /
            'urdf/monodrone_interceptor.urdf'
        ),
        description='URDF file for interceptor drone'
    )

    declare_urdf_target = DeclareLaunchArgument(
        'urdf_target',
        default_value=str(
            Path(get_package_share_directory('gz_sim_one')) /
            'urdf/monodrone_target.urdf'
        ),
        description='URDF file for target drone'
    )

    # =============================
    # Generic RSP launcher
    # =============================
    def launch_rsp(context, urdf_cfg, node_name):
        urdf_path = urdf_cfg.perform(context)

        if not os.path.isfile(urdf_path):
            raise RuntimeError(f"URDF file not found: {urdf_path}")

        with open(urdf_path, 'r') as f:
            urdf_content = f.read()

        return Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name=node_name,
            parameters=[
                {'robot_description': urdf_content},
                {'use_sim_time': True}
            ],
            output='screen'
        )

    # =============================
    # Launch both robots
    # =============================
    def launch_all_rsp(context, *args, **kwargs):
        return [
            launch_rsp(context, urdf_interceptor, 'rsp_interceptor'),
            launch_rsp(context, urdf_target, 'rsp_target'),
        ]

    # =============================
    # ros_gz bridge
    # =============================
    bridge_node = ExecuteProcess(
        cmd=[
            FindExecutable(name='ros2'),
            'run', 'ros_gz_bridge', 'parameter_bridge',
            '--ros-args',
            '-p', 'config_file:=src/gz2ros/config/bridge_params_one.yaml',
            '-p', 'use_sim_time:=true'
        ],
        output='screen',
        shell=True
    )

    # =============================
    # Custom nodes
    # =============================
    pose_tf_node = Node(
        package='gz2ros',
        executable='tf_pub_node',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    pose_sensors_tf_node = Node(
        package='gz2ros',
        executable='tf_pub_sensors',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    process_lidar_node = Node(
        package='gz2ros',
        executable='process_lidar_node',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # =============================
    # Launch description
    # =============================
    ld = LaunchDescription()

    ld.add_action(declare_urdf_interceptor)
    ld.add_action(declare_urdf_target)
    ld.add_action(OpaqueFunction(function=launch_all_rsp))

    ld.add_action(bridge_node)
    ld.add_action(pose_sensors_tf_node)
    # ld.add_action(process_lidar_node)  # décommente si nécessaire

    return ld

