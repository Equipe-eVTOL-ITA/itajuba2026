#!/usr/bin/env python3
"""Launch de VOO REAL da missao fase1_itjbx (itajuba_2026)."""

import datetime
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('fase1_itjbx'), 'config', 'flight.yaml')

    # YAML do detector e' um pacote (e portanto um arquivo) separado do da
    # missao -- mesmo padrao do simulation.launch.py (ver o comentario la).
    vision_pkg_dir = get_package_share_directory('base_detector_itjbx2026')
    vision_params = os.path.join(vision_pkg_dir, 'config', 'base_detector_itjbx2026.yaml')

    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    bag_dir = os.path.expanduser(f'~/evtol/mission_logs/fase1_itjbx_{stamp}')

    # Grava um rosbag de cada voo. Depois de uma missao que deu errado, esta e
    # a unica forma de saber o que o drone via no momento.
    bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', bag_dir,
             '/rosout',
             '/drone_trajectory',
             '/telemetry/drone_status',
             '/base_detector_itjbx2026/detections',
             '/fmu/out/vehicle_local_position',
             '/fmu/out/vehicle_status',
             '/fmu/in/trajectory_setpoint'],
        output='screen')

    system_health = Node(
        package='drone_lib', executable='system_health',
        parameters=[params], output='screen')

    # Em voo a imagem vem do camera_publisher, e nao de uma ponte do Gazebo
    # (mesmo padrao do cbr2026/fase1 -- veja o comentario la). O topico
    # (webcam_publisher: camera_name 'vertical' -> /vertical_camera/compressed,
    # ver flight.yaml) e' o mesmo que o image_bridge da simulacao entrega, entao
    # o base_detector_itjbx2026.yaml nao muda entre os dois.
    camera = Node(
        package='camera_publisher', executable='webcam',
        parameters=[params], output='screen')

    vision = Node(
        package='base_detector_itjbx2026', executable='base_detector_itjbx2026',
        parameters=[vision_params], output='screen')

    mission = Node(
        package='fase1_itjbx', executable='fase1_itjbx',
        parameters=[params], output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Abrir o RViz2'),
        bag,
        camera,
        system_health,
        vision,
        # A FSM espera 5 s para os outros nos subirem antes de comecar. Sem
        # isso ela decola antes de o detector estar pronto e varre o primeiro
        # trecho da grade cega.
        TimerAction(period=5.0, actions=[mission]),
    ])
