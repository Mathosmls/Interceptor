from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='serial:///dev/ttyUSB0:115200',
        description='FCU connection URL')

    traj_amplitude_arg = DeclareLaunchArgument(
        'traj_amplitude', default_value='3.0')

    traj_wavelength_arg = DeclareLaunchArgument(
        'traj_wavelength', default_value='10.0')

    traj_speed_arg = DeclareLaunchArgument(
        'traj_forward_speed', default_value='1.0')

    pkg_share = FindPackageShare('nmpc_rover')
    config_file = PathJoinSubstitution([pkg_share, 'config', 'nmpc_params.yaml'])

    mavros_node = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'mavros', 'mavros_node',
            '--ros-args',
            '-p', ['fcu_url:=', LaunchConfiguration('fcu_url')],
            '-p', 'gcs_url:=udp://@localhost:14550',
        ],
        output='screen',
    )

    nmpc_node = Node(
        package='nmpc_rover',
        executable='nmpc_controller',
        name='nmpc_controller',
        output='screen',
        parameters=[
            config_file,
            {
                'traj_amplitude': LaunchConfiguration('traj_amplitude'),
                'traj_wavelength': LaunchConfiguration('traj_wavelength'),
                'traj_forward_speed': LaunchConfiguration('traj_forward_speed'),
            },
        ],
    )

    return LaunchDescription([
        fcu_url_arg,
        traj_amplitude_arg,
        traj_wavelength_arg,
        traj_speed_arg,
        LogInfo(msg='Lancement MAVROS + NMPC Controller'),
        mavros_node,
        nmpc_node,
    ])
