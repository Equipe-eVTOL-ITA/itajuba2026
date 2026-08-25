#!/usr/bin/env python3
"""Launch de VOO REAL da missao fase2_itjbx (itajuba_2026)."""

import datetime
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
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
             '/fmu/out/vehicle_local_position',
             '/fmu/out/vehicle_status',
             '/fmu/in/trajectory_setpoint'],
        output='screen')

    system_health = Node(
        package='drone_lib', executable='system_health',
        parameters=[params], output='screen')

    mission = Node(
        package='fase2_itjbx', executable='fase2_itjbx',
        parameters=[params], output='screen')

    # ACRESCENTE aqui os nos de visao desta missao, ex.:
    # vision = Node(package='cv_nodes_algum', executable='detector', output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Abrir o RViz2'),
        bag,
        system_health,
        # A FSM espera 5 s para os outros nos subirem antes de comecar.
        TimerAction(period=5.0, actions=[mission]),
    ])
