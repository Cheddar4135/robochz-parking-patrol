"""
sim.launch.py — STEP3 인프라 (한 번 띄우고 유지)
  - world.launch.py : 우리 주차장 world + TB3 spawn
  - Nav2 : map_server(우리 parking_lot 맵) + AMCL(자동 initial pose) + planner/controller/bt
  순찰 노드(patrol/perception/monitor)는 app.launch.py 에서 별도로.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    bringup = get_package_share_directory('robochz_bringup')
    tb3_nav2 = get_package_share_directory('turtlebot3_navigation2')

    world_launch = os.path.join(bringup, 'launch', 'world.launch.py')
    nav2_launch = os.path.join(tb3_nav2, 'launch', 'navigation2.launch.py')

    map_yaml = os.path.join(bringup, 'maps', 'parking_lot.yaml')        # ground-truth 맵
    nav2_params = os.path.join(bringup, 'config', 'nav2_waffle.yaml')   # AMCL set_initial_pose 포함

    # 우리 world + TB3 (gazebo/spawn/rsp)
    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(world_launch))

    # Nav2 (use_sim_time:=True 가 yaml 의 use_sim_time:False 를 RewrittenYaml 로 전부 덮어씀)
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={
            'use_sim_time': 'True',
            'map': map_yaml,
            'params_file': nav2_params,
        }.items())

    return LaunchDescription([world, nav2])
