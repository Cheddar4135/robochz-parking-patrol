"""
app.launch.py — 우리 노드만 띄움 (sim.launch.py가 별도 터미널에 떠 있어야 함)
  - alert_node, perception_node, patrol_node, path_recorder, dynamic_car_mover
  - monitor_node 는 ★ 별도 터미널에서 실행 (로그 분리 위해 app.launch 에서 제외):
      ros2 run robochz_monitor monitor_node --ros-args -p use_sim_time:=true
  - patrol_node 는 config/patrol_waypoints.yaml 의 순찰 좌표를 파라미터로 받음
  - 코드 수정 후 재실행 시 sim 환경은 유지하고 이 launch만 Ctrl+C / 재실행.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory('robochz_bringup')
    waypoints = os.path.join(bringup, 'config', 'patrol_waypoints.yaml')
    dyn_car = os.path.join(bringup, 'config', 'dynamic_car_path.yaml')

    sim_time = {'use_sim_time': True}

    # monitor_node 는 별도 터미널에서 실행(로그 분리) → 위 docstring 참고
    alert = Node(package='robochz_monitor', executable='alert_node',
                 output='screen', parameters=[sim_time])   # 미등록 즉시 알림 GUI
    perception = Node(package='robochz_perception', executable='perception_node',
                      output='screen', parameters=[sim_time])
    patrol = Node(package='robochz_patrol', executable='patrol_node',
                  output='screen', parameters=[waypoints])   # waypoints.yaml 에 use_sim_time 포함
    path_rec = Node(package='robochz_patrol', executable='path_recorder',
                    output='screen', parameters=[sim_time])   # /patrol_path 기록·시각화
    dyn = Node(package='robochz_bringup', executable='dynamic_car_mover',
               output='screen', parameters=[dyn_car])         # 고정경로 동적 차량(Iter3 B)

    return LaunchDescription([alert, perception, patrol, path_rec, dyn])
