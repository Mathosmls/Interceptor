"""
Launch complet : MAVROS + Contrôleur NMPC pour rover ArduPilot réel

Usage :
  ros2 launch nmpc_rover nmpc_rover.launch.py

Arguments optionnels :
  fcu_url:=serial:///dev/ttyACM0:115200   (port série ArduPilot)
  gcs_url:=udp://:14550@localhost:14551   (GCS optionnel)
  traj_amplitude:=3.0
  traj_wavelength:=10.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():

    # ── Arguments ──────────────────────────────────────────────────────
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='serial:///dev/ttyACM0:115200',
        description='FCU connection URL (ArduPilot via USB/serial/UDP)'
        # Exemples :
        #   serial:///dev/ttyACM0:115200   (USB direct)
        #   serial:///dev/ttyS0:57600      (UART)
        #   udp://:14550@192.168.1.1:14555 (WiFi/Ethernet)
        #   tcp://192.168.1.1:5760         (TCP)
    )

    gcs_url_arg = DeclareLaunchArgument(
        'gcs_url',
        default_value='udp://@localhost:14550',
        description='URL de la GCS (Mission Planner / QGroundControl)'
    )

    traj_amplitude_arg = DeclareLaunchArgument(
        'traj_amplitude', default_value='3.0',
        description='Amplitude sinusoïdale (m)')

    traj_wavelength_arg = DeclareLaunchArgument(
        'traj_wavelength', default_value='10.0',
        description='Longueur d\'onde sinusoïdale (m)')

    traj_speed_arg = DeclareLaunchArgument(
        'traj_forward_speed', default_value='1.0',
        description='Vitesse de progression nominale (m/s)')

    # ── Chemin config ──────────────────────────────────────────────────
    pkg_share = FindPackageShare('nmpc_rover')
    config_file = PathJoinSubstitution([pkg_share, 'config', 'nmpc_params.yaml'])

    # ── Node MAVROS ────────────────────────────────────────────────────
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        namespace='mavros',
        output='screen',
        parameters=[
            {
                'fcu_url': LaunchConfiguration('fcu_url'),
                'gcs_url': LaunchConfiguration('gcs_url'),
                'target_system_id': 1,
                'target_component_id': 1,
                'fcu_protocol': 'v2.0',
                # Plugins à activer (minimaux pour rover)
                'plugin_allowlist': [
                    'command',
                    'global_position',
                    'local_position',
                    'param',
                    'rc_io',
                    'setpoint_velocity',
                    'sys_status',
                    'waypoint',
                ],
            }
        ],
    )

    # ── Node NMPC ──────────────────────────────────────────────────────
    nmpc_node = Node(
        package='nmpc_rover',
        executable='nmpc_controller',
        name='nmpc_controller',
        output='screen',
        parameters=[
            config_file,
            {
                # Surcharge depuis les arguments de launch
                'traj_amplitude':     LaunchConfiguration('traj_amplitude'),
                'traj_wavelength':    LaunchConfiguration('traj_wavelength'),
                'traj_forward_speed': LaunchConfiguration('traj_forward_speed'),
            }
        ],
    )

    # ── Optionnel : Rviz2 ──────────────────────────────────────────────
    # Décommentez si vous souhaitez visualiser la trajectoire
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    # )

    return LaunchDescription([
        fcu_url_arg,
        gcs_url_arg,
        traj_amplitude_arg,
        traj_wavelength_arg,
        traj_speed_arg,
        LogInfo(msg='🚀 Lancement MAVROS + NMPC Controller'),
        mavros_node,
        nmpc_node,
        # rviz_node,
    ])
