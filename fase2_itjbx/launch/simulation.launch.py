#!/usr/bin/env python3
"""Launch de SIMULACAO da missao fase2_itjbx (itajuba_2026)."""

import datetime
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('fase2_itjbx')
    params = os.path.join(pkg_dir, 'config', 'simulation.yaml')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'trajectory.rviz')

    # PLACEHOLDER copiado da fase1_itjbx: sobe o MESMO detector da fase 1
    # (base_detector_itjbx2026). So funciona de verdade se a fase 2 precisar
    # detectar a mesma coisa que a fase 1 -- troque para o pacote/YAML de
    # visao proprios da fase 2 assim que existirem.
    #
    # YAML do detector e' um pacote (e portanto um arquivo) separado do da
    # missao -- ver cv_nodes/detector/README.md, "Por que separado?". Um
    # detector chamado base_detector_itjbx2026 nao encontra chave nenhuma no
    # simulation.yaml da missao (que e' keyed em fase2_itjbx_node:) e usaria
    # os defaults do codigo em silencio.
    vision_pkg_dir = get_package_share_directory('base_detector_itjbx2026')
    vision_params = os.path.join(vision_pkg_dir, 'config', 'base_detector_itjbx2026.yaml')

    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    bag_dir = os.path.expanduser(f'~/evtol/mission_logs/fase2_itjbx_{stamp}')

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

    # Ponte de imagem do Gazebo para o ROS -- sem ela o base_detector_itjbx2026
    # sobe, nao reclama de nada e simplesmente nunca recebe quadro. Mesmo
    # padrao do cbr2026/fase1 (veja o comentario la).
    image_bridge = Node(
        package='ros_gz_image', executable='image_bridge',
        arguments=['/vertical_camera'],
        output='screen')

    vision = Node(
        package='base_detector_itjbx2026', executable='base_detector_itjbx2026',
        parameters=[vision_params], output='screen')

    mission = Node(
        package='fase2_itjbx', executable='fase2_itjbx',
        parameters=[params], output='screen')

    rviz = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Abrir o RViz2 com a trajetoria do drone'),
        bag,
        system_health,
        image_bridge,
        vision,
        rviz,
        # A FSM espera 5 s para os outros nos subirem antes de comecar. Sem
        # isso ela decola antes de o detector estar pronto e varre o primeiro
        # trecho da grade cega.
        TimerAction(period=5.0, actions=[mission]),
    ])
