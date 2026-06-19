"""
sim_rulebased.launch.py — 시뮬 인프라 (Rule-based 주행 데모용). app_rulebased 와 짝.
  - world.launch.py : 우리 주차장 world + Cheezlbot spawn
  - map→odom       : patrol_rulebased 가 첫 odom 보정해 발행(이중가산 자동 해소) → 여기선 안 띄움
  - map_server     : RViz 맵 표시용 (/map). Nav2 컨트롤러는 안 띄움
                     (patrol_rulebased 가 /cmd_vel 을 직접 내보내므로 충돌 방지).
  순찰 노드(patrol_rulebased/perception/monitor)는 app_rulebased.launch.py 에서.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory('robochz_bringup')

    world_launch = os.path.join(bringup, 'launch', 'world.launch.py')
    map_yaml = os.path.join(bringup, 'maps', 'parking_lot.yaml')

    # 우리 world + 로봇 (gazebo/spawn/rsp)
    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(world_launch))

    # map_server (RViz 맵 표시) + 라이프사이클 자동 활성
    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        output='screen',
        parameters=[{'use_sim_time': True, 'yaml_filename': map_yaml}])
    lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_map', output='screen',
        parameters=[{'use_sim_time': True, 'autostart': True,
                     'node_names': ['map_server']}])

    return LaunchDescription([world, map_server, lifecycle])
