"""
world.launch.py — 우리 주차장 world + TurtleBot3 spawn (Iteration 1, STEP1~2용)
  - parking_lot.world (Distribution_Warehouse + 주차라인) 로드
  - TB3 robot_state_publisher + spawn
  - Nav2 는 아직 없음 (맵 생성 후 STEP3에서 sim.launch.py 로 재구성)
"""
import os

os.environ.setdefault('TURTLEBOT3_MODEL', 'waffle')

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory('robochz_bringup')
    gazebo_ros = get_package_share_directory('gazebo_ros')
    tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    world = os.path.join(bringup, 'worlds', 'parking_lot.world')
    models_dir = os.path.join(bringup, 'models')   # model://Distribution_Warehouse 해석용

    # TB3 gazebo model.sdf — diff_drive(/cmd_vel·/odom·TF) + ray_sensor(/scan) 플러그인 포함.
    # (URDF/robot_description 에는 이 플러그인이 없어 -topic 스폰은 로봇이 안 움직임)
    tb3_sdf = os.path.join(
        tb3_gazebo, 'models',
        'turtlebot3_' + os.environ['TURTLEBOT3_MODEL'], 'model.sdf')

    # 기존 GAZEBO_MODEL_PATH 에 우리 models 를 "추가" (sun/ground_plane 기본 경로 보존)
    # include 평가 전 즉시 반영(부모 process) + SetEnvironmentVariable(자식 process) 이중 보강
    model_path = models_dir + os.pathsep + os.environ.get('GAZEBO_MODEL_PATH', '')
    os.environ['GAZEBO_MODEL_PATH'] = model_path

    # spawn 위치 = A구역 통로 중심 (깨끗한 구역)
    spawn_x, spawn_y = '11.85', '8.0'

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world, 'verbose': 'true'}.items())

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')))

    rsp = IncludeLaunchDescription(   # TB3 URDF 를 /robot_description 으로 발행
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo, 'launch', 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items())

    spawn = Node(   # model.sdf(diff_drive·laser 플러그인 포함)로 TB3 스폰
        package='gazebo_ros', executable='spawn_entity.py', output='screen',
        arguments=['-entity', 'waffle', '-file', tb3_sdf,
                   '-x', spawn_x, '-y', spawn_y, '-z', '0.01'])

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path),
        gzserver, gzclient, rsp, spawn,
    ])
