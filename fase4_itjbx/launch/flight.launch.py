#!/usr/bin/env python3
"""Launch de VOO REAL da missao fase4_itjbx (itajuba_2026)."""

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
    params = os.path.join(pkg_dir, 'config', 'flight.yaml')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'trajectory.rviz')

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

    # Em voo a imagem vem do camera_publisher, e nao de uma ponte do Gazebo
    # (mesmo padrao do cbr2026/fase1 -- veja o comentario la). O topico
    # (webcam_publisher: camera_name 'vertical' -> /vertical_camera/compressed,
    # ver flight.yaml) e' o mesmo que os tres detectores ja esperam
    # (image_topic em lane_detector.yaml, circle_detector.yaml e
    # red_line_detector.yaml), entao nenhum deles muda entre sim e voo.
    camera = Node(
        package='camera_publisher', executable='webcam',
        parameters=[params], output='screen')

    lane_detector = Node(
        package='lane_detector', executable='lane_detector_node',
        parameters=[lane_params], output='screen')

    circle_detector = Node(
        package='circle_detector', executable='circle_detector_node',
        parameters=[circle_params], output='screen')

    # Precisa rodar desde o inicio -- ver comentario em simulation.launch.py.
    red_line_detector = Node(
        package='red_line_detector', executable='red_line_detector_node',
        parameters=[red_line_params], output='screen')

    mission = Node(
        package='fase4_itjbx', executable='fase4_itjbx',
        parameters=[params], output='screen')

    # Mesmo padrao do simulation.launch.py -- o argumento 'rviz' so' existia
    # de nome aqui antes (declarado, nunca usado).
    rviz = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Abrir o RViz2'),
        bag,
        system_health,
        camera,
        lane_detector,
        circle_detector,
        red_line_detector,
        rviz,
        # A FSM espera 5 s para os nos de visao subirem antes de comecar.
        TimerAction(period=5.0, actions=[mission]),
    ])
