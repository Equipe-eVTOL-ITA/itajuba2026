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

    # Ponte de imagem do Gazebo para o ROS -- sem ela o RDPformas sobe, nao
    # reclama de nada e simplesmente nunca recebe quadro. Mesmo padrao do
    # fase1_itjbx/cbr2026-fase1 (veja o comentario la).
    image_bridge = Node(
        package='ros_gz_image', executable='image_bridge',
        arguments=['/vertical_camera'],
        output='screen')

    # No de visao: mesmo detector que sae2026/mission_1 usa (ver a conversa
    # que motivou o port desta missao) -- publica ArUco + forma + bases
    # numeradas em "bouncing_detection". debug_mask=True SO' AQUI (na
    # simulacao) -- liga o publisher de 'bouncing_detection_mask/compressed'
    # (a mascara que de fato vai pro findContours) pro mask_view abaixo;
    # flight.launch.py nao passa esse parametro, entao no voo real o
    # publisher de mascara fica desligado (Detector.debug_mask default e'
    # False) e nao gasta CPU/banda a toa.
    vision = Node(
        package='RDPformas', executable='RDPformas', output='screen',
        parameters=[{'debug_mask': True}])

    mission = Node(
        package='fase2_itjbx', executable='fase2_itjbx',
        parameters=[params], output='screen')

    rviz = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen')

    # Janela com a mascara que o RDPformas usa pra achar contornos (ver
    # 'debug_mask' acima) -- pedido explicito pra debugar deteccao de
    # forma/ArUco ao vivo, na simulacao. rqt_image_view espera o topico
    # BASE (sem o sufixo /compressed) + o parametro _image_transport,
    # senao tenta se inscrever direto no topico com transporte errado e
    # nunca recebe frame (ver aviso do proprio image_transport).
    mask_view = Node(
        package='rqt_image_view', executable='rqt_image_view',
        arguments=['/bouncing_detection_mask'],
        parameters=[{'_image_transport': 'compressed'}],
        condition=IfCondition(LaunchConfiguration('debug_gui')),
        output='screen')

    # Sliders com os parametros de deteccao de forma mais importantes (ver
    # RDPformas.py: min_parent_area, triangle/hexagon/star_circularity_*,
    # canny_threshold_ratio etc.) -- rqt_reconfigure so' desenha slider pra
    # parametro com range declarado, e' por isso que RDPformas.py declara
    # cada um com FloatingPointRange/IntegerRange em vez de so' um valor
    # solto. 'rdpvisao_node' vai como argumento posicional pra' ja' abrir
    # direto nos parametros do no de visao, sem precisar escolher no
    # dropdown (rqt_reconfigure aceita "node_name" -- ver --help).
    param_sliders = Node(
        package='rqt_reconfigure', executable='rqt_reconfigure',
        arguments=['rdpvisao_node'],
        condition=IfCondition(LaunchConfiguration('debug_gui')),
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Abrir o RViz2 com a trajetoria do drone'),
        DeclareLaunchArgument('debug_gui', default_value='true',
                              description='Abrir rqt_image_view (mascara) + rqt_reconfigure '
                                          '(sliders) automaticamente -- SO simulacao'),
        bag,
        system_health,
        image_bridge,
        vision,
        rviz,
        # 2s pro RDPformas subir e registrar seus parametros/topicos antes
        # das ferramentas de debug tentarem se conectar -- sem isso o
        # rqt_reconfigure as vezes abre sem "rdpvisao_node" na lista e exige
        # um refresh manual.
        TimerAction(period=2.0, actions=[mask_view, param_sliders]),
        # A FSM espera 5 s para os outros nos subirem antes de comecar. Sem
        # isso ela decola antes de o detector estar pronto e varre o primeiro
        # trecho da grade cega.
        TimerAction(period=5.0, actions=[mission]),
    ])
