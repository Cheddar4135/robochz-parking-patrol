"""
app.launch.py — 우리 노드만 띄움 (sim.launch.py가 별도 터미널에 떠 있어야 함)
  - monitor_node, perception_node, patrol_node
  - 코드 수정 후 재실행 시 sim 환경은 유지하고 이 launch만 Ctrl+C / 재실행.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    monitor = Node(package='robochz_monitor',     executable='monitor_node',     output='screen')
    perception = Node(package='robochz_perception', executable='perception_node', output='screen')
    patrol = Node(package='robochz_patrol',       executable='patrol_node',       output='screen')

    return LaunchDescription([monitor, perception, patrol])
