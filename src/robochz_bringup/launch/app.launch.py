"""
app.launch.py — 우리 노드만 띄움 (sim.launch.py가 별도 터미널에 떠 있어야 함)
  - monitor_node, perception_node, patrol_node
  - patrol_node 는 config/patrol_waypoints.yaml 의 순찰 좌표를 파라미터로 받음
  - 코드 수정 후 재실행 시 sim 환경은 유지하고 이 launch만 Ctrl+C / 재실행.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    waypoints = os.path.join(
        get_package_share_directory('robochz_bringup'),
        'config', 'patrol_waypoints.yaml')

    sim_time = {'use_sim_time': True}

    monitor = Node(package='robochz_monitor', executable='monitor_node',
                   output='screen', parameters=[sim_time])
    perception = Node(package='robochz_perception', executable='perception_node',
                      output='screen', parameters=[sim_time])
    patrol = Node(package='robochz_patrol', executable='patrol_node',
                  output='screen', parameters=[waypoints])   # waypoints.yaml 에 use_sim_time 포함
    path_rec = Node(package='robochz_patrol', executable='path_recorder',
                    output='screen', parameters=[sim_time])   # /patrol_path 기록·시각화

    return LaunchDescription([monitor, perception, patrol, path_rec])
