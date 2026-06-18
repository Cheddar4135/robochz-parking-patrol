"""
sim.launch.py — 시뮬레이션 환경(한 번만 띄우고 유지)
  - TurtleBot3 Gazebo (외곽 spawn)
  - Nav2 스택 + RViz (waffle 파라미터, turtlebot3_world 맵)
  - AMCL initial pose 자동 셋업 (spawn 위치와 일치)
우리 노드는 별도 app.launch.py 에서.
"""
import os

os.environ.setdefault('TURTLEBOT3_MODEL', 'waffle')

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    tb3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')
    tb3_nav2_share = get_package_share_directory('turtlebot3_navigation2')

    tb3_world_launch = os.path.join(tb3_gazebo_share, 'launch', 'turtlebot3_world.launch.py')
    nav2_launch = os.path.join(tb3_nav2_share, 'launch', 'navigation2.launch.py')
    # 중요: turtlebot3_navigation2 의 map.yaml 은 turtlebot3_world.world 와 짝이 아닌
    # 다른 SLAM 결과(md5 불일치). nav2_bringup 의 turtlebot3_world.yaml 이 정확한 짝.
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    map_yaml_path = os.path.join(nav2_bringup_share, 'maps', 'turtlebot3_world.yaml')
    nav2_params_path = os.path.join(tb3_nav2_share, 'param', 'humble', 'waffle.yaml')

    # spawn 위치 = AMCL initial pose
    spawn_x = '-2.0'
    spawn_y = '0.5'

    set_tb3_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle')

    tb3_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tb3_world_launch),
        launch_arguments={'x_pose': spawn_x, 'y_pose': spawn_y}.items()
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={
            'use_sim_time': 'True',
            'map': map_yaml_path,
            'params_file': nav2_params_path,
        }.items()
    )

    # 자동 initial pose 셋업은 AMCL active 타이밍 의존이라 깨지기 쉬움.
    # Iteration 0에서는 RViz에서 수동으로 2D Pose Estimate (LaserScan 정합 기준)이 더 견고.
    # Iteration 1+에서 AMCL params yaml의 set_initial_pose: true 로 더 정확하게 자동화 예정.

    return LaunchDescription([
        set_tb3_model,
        tb3_gazebo,
        nav2,
    ])
