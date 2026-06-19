"""
app_rulebased.launch.py — Rule-based 주행 노드들 (sim_rulebased.launch.py 와 짝).
  - monitor_node, perception_node, patrol_rulebased(odom 기반 /cmd_vel 직접제어)
  - patrol_rulebased 는 config/patrol_waypoints.yaml 좌표를 파라미터로 받음
  - Nav2 없이 동작. 데모 재현용.
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
    alert = Node(package='robochz_monitor', executable='alert_node',
                 output='screen', parameters=[sim_time])   # 미등록 즉시 알림 GUI
    perception = Node(package='robochz_perception', executable='perception_node',
                      output='screen', parameters=[sim_time])
    patrol = Node(package='robochz_patrol', executable='patrol_rulebased',
                  output='screen', parameters=[waypoints])   # waypoints.yaml 에 use_sim_time 포함
    path_rec = Node(package='robochz_patrol', executable='path_recorder',
                    output='screen', parameters=[sim_time])   # /patrol_path 기록·시각화

    return LaunchDescription([monitor, alert, perception, patrol, path_rec])
