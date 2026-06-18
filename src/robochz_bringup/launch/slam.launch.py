"""
slam.launch.py — slam_toolbox online async 매핑 (Iteration 1, STEP2)
  world.launch.py 로 sim+TB3 띄운 뒤 별도로 실행해 지도를 작성한다.
  /scan + /odom + TF(odom→base_footprint) → /map + TF(map→odom)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('robochz_bringup'),
        'config', 'mapper_params_online_async.yaml')

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',   # online async 매핑 노드
        name='slam_toolbox',
        output='screen',
        parameters=[params, {'use_sim_time': True}])  # yaml + 런타임 보강

    return LaunchDescription([slam])
