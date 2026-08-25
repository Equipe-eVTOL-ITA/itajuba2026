#!/usr/bin/env python3
"""Launch de SIMULACAO da missao fase4_itjbx (itajuba_2026)."""

import datetime
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('fase4_itjbx')
    params = os.path.join(pkg_dir, 'config', 'simulation.yaml')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'trajectory.rviz')

    # YAML dos detectores e' um pacote (e portanto um arquivo) separado do da
    # missao -- mesmo padrao do fase1_itjbx com o base_detector_itjbx2026.
    lane_pkg_dir = get_package_share_directory('lane_detector')
    lane_params = os.path.join(lane_pkg_dir, 'config', 'lane_detector.yaml')
    circle_pkg_dir = get_package_share_directory('circle_detector')
    circle_params = os.path.join(circle_pkg_dir, 'config', 'circle_detector.yaml')
    red_line_pkg_dir = get_package_share_directory('red_line_detector')
    red_line_params = os.path.join(red_line_pkg_dir, 'config', 'red_line_detector.yaml')

    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    bag_dir = os.path.expanduser(f'~/evtol/mission_logs/fase4_itjbx_{stamp}')

    # Grava um rosbag de cada voo. Depois de uma missao que deu errado, esta e
    # a unica forma de saber o que o drone via no momento.
    bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', bag_dir,
             '/rosout',
             '/drone_trajectory',
             '/telemetry/drone_status',
             '/lane_detection',
             '/base_circle',
             '/red_line_detection',
             '/fmu/out/vehicle_local_position',
             '/fmu/out/vehicle_status',
             '/fmu/in/trajectory_setpoint'],
        output='screen')

    system_health = Node(
        package='drone_lib', executable='system_health',
        parameters=[params], output='screen')

    # Ponte de imagem do Gazebo para o ROS -- sem ela os detectores sobem, nao
    # reclamam de nada e simplesmente nunca recebem quadro. Mesmo padrao do
    # fase1_itjbx (veja o comentario la).
    image_bridge = Node(
        package='ros_gz_image', executable='image_bridge',
        arguments=['/vertical_camera'],
        output='screen')

    # No de visao: segue a linha azul e publica /lane_detection (custom_msgs/msg/LaneDirection)
    lane_detector = Node(
        package='lane_detector', executable='lane_detector_node',
        parameters=[lane_params], output='screen')

    # No separado: acha a base circular via HoughCircles e por onde a linha sai dela,
    # publica /base_circle. So precisa rodar no comeco da missao (enquanto a base ainda
    # esta a vista) -- se auto-encerra sozinho depois de active_duration_s.
    circle_detector = Node(
        package='circle_detector', executable='circle_detector_node',
        parameters=[circle_params], output='screen')

    # No de visao: acha a mangueira vermelha entre os postes e publica
    # /red_line_detection. PRECISA rodar desde o inicio da missao (nao so a partir
    # de ALIGN_RED_LINE) -- e' o FollowLineState que monitora esse topico pra saber
    # a hora de parar de seguir a linha azul e trocar de estado (ver
    # follow_line_state.hpp); se esse no' subisse so depois, o primeiro quadro em
    # que a mangueira aparece nunca seria visto.
    red_line_detector = Node(
        package='red_line_detector', executable='red_line_detector_node',
        parameters=[red_line_params], output='screen')

    mission = Node(
        package='fase4_itjbx', executable='fase4_itjbx',
        parameters=[params], output='screen')

    rviz = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen')

    # Mostra a imagem da camera anotada pelo lane_detector (pontos das regioes +
    # reta ajustada) -- e' o topico mais direto pra saber se a deteccao esta
    # funcionando sem abrir o rqt na mao e escolher entre debug/mask/mask_overlay
    # a cada simulacao.
    #
    # Topico BASE (/lane_detector/debug) + parametro _image_transport:=compressed,
    # NAO o topico ".../compressed" direto -- passar o topico compressed direto
    # faz o proprio image_transport avisar "you will likely get a connection
    # error" (rode `ros2 run rqt_image_view rqt_image_view <topico>/compressed`
    # pra ver o aviso), e essa subscricao malformada e' o motivo mais provavel do
    # crash: sem sintoma nenhum enquanto nao chega frame nenhum, mas quebra assim
    # que o lane_detector comeca a publicar de verdade -- exatamente "crash no
    # inicio da missao".
    #
    # QT_QPA_PLATFORM=xcb mantido como seguranca extra: a sessao desta maquina e'
    # Wayland nativo, e o plugin Qt5 do Wayland e' historicamente instavel quando
    # ha' outra janela OpenGL pesada por perto (o proprio Gazebo). Rodando via
    # XWayland evita essa combinacao tambem, mesmo que a causa principal seja o
    # transport acima.
    rqt_debug_image = Node(
        package='rqt_image_view', executable='rqt_image_view',
        arguments=['/lane_detector/debug'],
        parameters=[{'_image_transport': 'compressed'}],
        additional_env={'QT_QPA_PLATFORM': 'xcb'},
        condition=IfCondition(LaunchConfiguration('rqt')),
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Abrir o RViz2 com a trajetoria do drone'),
        DeclareLaunchArgument('rqt', default_value='true',
                              description='Abrir o rqt_image_view no topico de debug do lane_detector'),
        bag,
        system_health,
        image_bridge,
        lane_detector,
        circle_detector,
        red_line_detector,
        rviz,
        # O rqt_image_view seleciona o topico do argumento de linha de comando
        # UMA vez, contra a lista de topicos que existir naquele instante -- se
        # o lane_detector ainda nao tiver anunciado /lane_detector/debug/compressed
        # (ele sobe junto, sem delay, ao mesmo tempo que o rqt), o topico nao
        # aparece selecionado e a janela abre em branco, sem tentar de novo depois.
        # 3s da folga suficiente pro image_bridge + lane_detector ja estarem
        # publicando quando o rqt inicializa.
        TimerAction(period=3.0, actions=[rqt_debug_image]),
        # A FSM espera 5 s para os nos de visao subirem e comecarem a publicar
        # antes de entrar em SEARCH_LINE.
        TimerAction(period=5.0, actions=[mission]),
    ])
