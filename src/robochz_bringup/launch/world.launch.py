"""
world.launch.py — 우리 주차장 world + Cheezlbot(robochz_description) spawn
  - parking_lot.world (Distribution_Warehouse + 주차라인 + 차량) 로드
  - robochz_description 의 urdf(robot_state_publisher) + model.sdf(spawn)
    → Iter2: 카메라 우측 90° 마운트 적용본 사용
  - Nav2 는 sim.launch.py 가 이 위에 얹음
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
    desc = get_package_share_directory('robochz_description')   # 우리 로봇(카메라 90°)

    world = os.path.join(bringup, 'worlds', 'parking_lot.world')
    models_dir = os.path.join(bringup, 'models')   # model://Distribution_Warehouse 해석용

    # 우리 로봇 description (Iter2 카메라 우측 90° 적용본)
    urdf_path = os.path.join(desc, 'urdf', 'robochz_waffle.urdf')   # TF용 (camera_link 회전)
    robot_sdf = os.path.join(desc, 'models', 'robochz_waffle', 'model.sdf')  # spawn용 (센서 회전 + 플러그인)
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    # GAZEBO_MODEL_PATH: 우리 models + TB3 common 메시(model://turtlebot3_common 해석)
    tb3_models = os.path.join(tb3_gazebo, 'models')
    model_path = os.pathsep.join(
        [models_dir, tb3_models, os.environ.get('GAZEBO_MODEL_PATH', '')])
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

    rsp = Node(   # 우리 urdf 를 /robot_description 으로 발행 (camera_link 90° 회전 반영)
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True, 'robot_description': robot_desc}])

    spawn = Node(   # 우리 model.sdf(diff_drive·laser·camera 플러그인 + 카메라 90°)로 스폰
        package='gazebo_ros', executable='spawn_entity.py', output='screen',
        arguments=['-entity', 'cheezlbot', '-file', robot_sdf,
                   '-x', spawn_x, '-y', spawn_y, '-z', '0.01'])

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path),
        gzserver, gzclient, rsp, spawn,
    ])
