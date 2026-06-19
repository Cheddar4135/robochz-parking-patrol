"""
sim.launch.py — 시뮬 인프라 (Nav2 + 정적 map→odom 디버그 버전). app.launch.py 와 짝.
  - world.launch.py : 우리 주차장 world + Cheezlbot spawn
  - Nav2           : map_server(우리 map) + AMCL(tf_broadcast:false) + planner/controller/bt
  - map_odom_calib : 첫 odom 보정해 정적 map→odom 발행 (이중가산 자동 해소, AMCL TF 대체)
  순찰 노드(patrol/perception/monitor)는 app.launch.py 에서 별도로.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory('robochz_bringup')
    tb3_nav2 = get_package_share_directory('turtlebot3_navigation2')

    world_launch = os.path.join(bringup, 'launch', 'world.launch.py')
    nav2_launch = os.path.join(tb3_nav2, 'launch', 'navigation2.launch.py')
    map_yaml = os.path.join(bringup, 'maps', 'parking_lot.yaml')
    nav2_params = os.path.join(bringup, 'config', 'nav2_waffle.yaml')

    # 우리 world + 로봇 (gazebo/spawn/rsp)
    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(world_launch))

    # Nav2 (use_sim_time:=True 가 yaml 의 False 를 RewrittenYaml 로 덮음)
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={
            'use_sim_time': 'True',
            'map': map_yaml,
            'params_file': nav2_params,
        }.items())

    # 정적 map→odom (AMCL TF 대체). 첫 odom 을 기준점으로 보정 → 이중가산 자동 해소.
    map_odom = Node(
        package='robochz_patrol', executable='map_odom_calib',
        name='map_odom_calib', output='screen',
        parameters=[{'use_sim_time': True, 'spawn_x': 11.85, 'spawn_y': 8.0}])

    return LaunchDescription([world, nav2, map_odom])
