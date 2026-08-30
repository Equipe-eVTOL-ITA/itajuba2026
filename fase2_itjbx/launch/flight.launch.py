#!/usr/bin/env python3
"""Launch de VOO REAL da missao fase2_itjbx (itajuba_2026)."""

import datetime
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('fase2_itjbx'), 'config', 'flight.yaml')

    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    bag_dir = os.path.expanduser(f'~/evtol/mission_logs/fase2_itjbx_{stamp}')

    # Grava um rosbag de cada voo. Depois de uma missao que deu errado, esta e
    # a unica forma de saber o que o drone via no momento.
    bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', bag_dir,
             '/rosout',
             '/drone_trajectory',
             '/telemetry/drone_status',
             '/bouncing_detection',
             '/discovered_bases',
             '/fmu/out/vehicle_local_position',
             '/fmu/out/vehicle_status',
             '/fmu/in/trajectory_setpoint'],
        output='screen')

    system_health = Node(
        package='drone_lib', executable='system_health',
        parameters=[params], output='screen')

    # Em voo a imagem vem do camera_publisher, e nao de uma ponte do Gazebo
    # (mesmo padrao do fase1_itjbx/fase4_itjbx -- veja o comentario la). O
    # topico (webcam_publisher: camera_name 'vertical' ->
    # /vertical_camera/compressed, ver flight.yaml) e' o mesmo nome que o
    # image_bridge da simulacao entrega -- por isso o CODIGO de deteccao
    # (RDPformas) nao muda entre os dois. SEM este no, RDPformas sobe, nao
    # reclama de nada e simplesmente nunca recebe quadro (ver o mesmo aviso
    # em simulation.launch.py sobre a ponte do Gazebo).
    camera = Node(
        package='camera_publisher', executable='webcam',
        parameters=[params], output='screen')

    # No de visao: mesmo detector que sae2026/mission_1 usa (ver a conversa
    # que motivou o port desta missao) -- publica ArUco + forma + bases
    # numeradas em "bouncing_detection", sem parametros proprios a passar.
    vision = Node(
        package='RDPformas', executable='RDPformas', output='screen')

    mission = Node(
        package='fase2_itjbx', executable='fase2_itjbx',
        parameters=[params], output='screen')

    return LaunchDescription([
        bag,
        camera,
        system_health,
        vision,
        # A FSM espera 5 s para os outros nos subirem antes de comecar. Sem
        # isso ela decola antes de o detector estar pronto e varre o primeiro
        # trecho da grade cega.
        TimerAction(period=5.0, actions=[mission]),
    ])
