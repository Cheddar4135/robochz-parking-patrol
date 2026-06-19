#!/usr/bin/env python3
"""주차라인 월드 생성기 — A/B/C 3구역 × (6×2=12칸) = 36칸.

사용자가 설계한 좌표를 파라미터로 받아, 흰색 주차선(충돌 없음) visual을
가진 Classic 호환 .world (sun + ground_plane + 창고 include + 주차라인)를 출력한다.
출력: ../worlds/parking_lot.world (이 스크립트 위치 기준).

칸 치수/좌표를 바꾸고 싶으면 아래 STALL_*, ZONES 만 수정하고 재실행하면 됨.
"""
import os

# ── 칸 치수 (사용자 설계) ──
STALL_W = 2.5      # 주차칸 폭
STALL_L = 5.0      # 주차칸 길이
LINE_W  = 0.1      # 주차선 폭
LINE_Z  = 0.02     # 라인을 바닥 위로 살짝 띄움 (z-fighting 방지)
LINE_H  = 0.01     # 라인 두께(높이)

# ── 구역별 설계 (사용자 제공 좌표) ──
#   stall_x : 6개 칸 중심 x
#   rows    : 두 줄의 중심 y (위/아래)
#   x_min/x_max : 구역 x 범위 (가로 경계선 길이 산출용)
ZONES = {
    'A': dict(stall_x=[5.35, 7.95, 10.55, 13.15, 15.75, 18.35],
              rows=[13.5, 2.5],   x_min=4.0,   x_max=19.7),
    'B': dict(stall_x=[-23.65, -21.05, -18.45, -15.85, -13.25, -10.65],
              rows=[-15.5, -26.5], x_min=-25.0, x_max=-9.3),
    'C': dict(stall_x=[5.35, 7.95, 10.55, 13.15, 15.75, 18.35],
              rows=[-15.5, -26.5], x_min=4.0,   x_max=19.7),
}

# ── 기둥 (AMCL localization 특징 + 정적 장애물) ──
# 통로 양옆(칸막이선 x × 통로경계 y)에 세워, 통로 주행 시 항상 3.5m 안에 특징 확보.
PILLAR_ZONES = ['A']   # 기둥 세울 구역 (Iter1은 A만; 확장 시 'B','C' 추가)
PILLAR_SIZE = 0.3      # 기둥 한 변 (m)
PILLAR_H = 2.5         # 기둥 높이 (m)

# ── 주차 차량 (Iter3: 실제 Car 메시, 5색) ──
# 외부 Car.dae(폭 X=2.1, 길이 Y=4.665, 바닥 z=0, front=-Y)를 5색 변형본으로.
# 메시는 visual, 충돌은 박스(mesh collision 은 LiDAR/costmap 부담 → 박스 유지).
# 번호판은 앞/뒤 얇은 visual(CarPlate/<라벨> = materials 의 plate_<라벨>.png).
MESH_W = 2.1        # 메시 폭 (X)
MESH_L = 4.7        # 메시 길이 (Y, 실측 4.665 ≈ 4.7)
CAR_H = 1.5         # 충돌박스 높이
CAR_Z = 0.75        # 충돌박스 중심 높이
PLATE_SIZE = (0.335, 0.01, 0.17)  # 번호판 판 (폭 x, 두께 y, 높이 z) — 기본 ±y 향함
PLATE_FRONT_Y = -2.40             # 앞 범퍼 표면(front=-Y)
PLATE_REAR_Y = 2.285              # 뒤 범퍼 표면(+Y)
PLATE_Z = 0.45
HALF_PI = 1.5708                  # 90° (rad)
PI = 3.14159

# 라벨: (중심x, 중심y, yaw, 색, 번호판) — A01~A12 중 10칸(빈칸 A06·A09).
#   yaw: 위줄(y13.5)=0 (메시 front −Y → 통로 −y) / 아래줄(y2.5)=π (front → 통로 +y).
#   등록 6(REGISTERED) / 미등록 4. 색 5종 골고루.
CARS = {
    'A01': (5.35,  13.5, 0.0, 'red',    '04자9332'),
    'A02': (7.95,  13.5, 0.0, 'blue',   '10마6654'),
    'A03': (10.55, 13.5, 0.0, 'white',  '10나0303'),
    'A04': (13.15, 13.5, 0.0, 'silver', '123가4568'),
    'A05': (15.75, 13.5, 0.0, 'yellow', '49다3433'),
    'A07': (5.35,  2.5,  PI,  'red',    '41라2500'),
    'A08': (7.95,  2.5,  PI,  'blue',   '24허1861'),
    'A10': (13.15, 2.5,  PI,  'white',  '40수3539'),
    'A11': (15.75, 2.5,  PI,  'silver', '76아9988'),
    'A12': (18.35, 2.5,  PI,  'yellow', '88오7175'),
}
REGISTERED = {'A01', 'A03', 'A04', 'A05', 'A08', 'A11'}   # 등록 6 (미등록 분산: A02·A07·A10·A12)

# ── 동적 차량 (Iter3: 고정경로 진입 차량) ──
# 주차차량과 동일 크기 박스. kinematic+무중력 → 물리에 안 휘둘리고 ROS 노드가
# /gazebo/set_entity_state 로 매 틱 텔레포트. 진짜 collision 이라 Classic LiDAR 가
# 그대로 감지 → Nav2 local costmap 회피. 색은 주황(침입 차량 구분).
# 스폰=경로 시작점(24,8.0), yaw=π(서쪽 향함). mover 가 cmd_vel 로 (24,8)→(11.85,8) 추종 후 정지.
# 동쪽서 역방향 진입 → 통로 중앙(로봇 스폰)에 멈춰 정지 장애물 → 로봇은 살짝 비켜감.
# 색 red, 번호판 CarPlate/DYN(94다3533, 미등록).
DYNAMIC_CAR = dict(name='dynamic_car', x=24.0, y=8.0, yaw=PI,
                   color='red', plate='DYN', plate_num='94다3533')

# ── 비대칭 랜드마크 (AMCL aliasing 방지) ──
# 통로가 x축으로 반복구조라 위치추정이 통로 따라 미끄러짐(aliasing) → 빈 주차칸 입구/끝에
# 서로 다른 사물을 비대칭 배치해 각 지점의 특징을 유일하게 만든다.
# 차량과 달리 영구 고정물이므로 map 에도 등록(gen_map_from_world 가 import).
#   name: (model_uri, x, y, yaw, map_sx, map_sy)  — map_sx/sy = 맵 래스터용 박스(회전 전 footprint)
LANDMARKS = {
    'barrier':  ('drc_practice_orange_jersey_barrier', 4.4,   7.0,  HALF_PI, 1.6, 0.6),  # 좌측 벽, y=7.0(동적차량 y=9.5 레인 비움)
    'shelf':    ('shelf',                              19.3,  12.5, HALF_PI, 3.6, 0.6),  # 우측 벽 상단 코너(A-06 빈칸)
    'ebox':     ('electrical_box',                     10.0,  10.7, 0.0,     0.7, 0.6),  # 통로 상단 가장자리(좌, A-03 빈칸)
    'stopsign': ('stop_sign',                          12.5,  10.7, 0.0,     0.3, 0.3),  # 통로 상단 가장자리(중, A-04 빈칸)
    'cone':     ('construction_cone',                  16.5,  5.3,  0.0,     0.5, 0.5),  # 통로 하단 가장자리(우, A-11 빈칸)
}


def line_visual(name, cx, cy, sx, sy):
    """바닥에 깔리는 흰색 라인 1개 (visual만, collision 없음)."""
    return f"""      <visual name="{name}">
        <pose>{cx:.3f} {cy:.3f} {LINE_Z} 0 0 0</pose>
        <geometry><box><size>{sx:.3f} {sy:.3f} {LINE_H}</size></box></geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
        </material>
      </visual>
"""


def zone_lines(tag, z):
    out = []
    sx = z['stall_x']
    # 세로 칸막이선 x중심: 양 끝단 + 칸 중심들의 중점 (7개)
    x_lines = [sx[0] - STALL_W / 2 - LINE_W / 2]
    x_lines += [(sx[i] + sx[i + 1]) / 2 for i in range(len(sx) - 1)]
    x_lines += [sx[-1] + STALL_W / 2 + LINE_W / 2]
    # 각 줄(row)마다 세로선 7개
    for r, ry in enumerate(z['rows']):
        for i, lx in enumerate(x_lines):
            out.append(line_visual(f"{tag}_r{r}_div{i}", lx, ry, LINE_W, STALL_L))
    # 가로 경계선: 각 줄의 앞/뒤 (y중심 ± 칸길이/2)
    x_center = (z['x_min'] + z['x_max']) / 2
    x_len = z['x_max'] - z['x_min']
    ybounds = sorted({round(ry + s * STALL_L / 2, 3)
                      for ry in z['rows'] for s in (-1, 1)})
    for j, by in enumerate(ybounds):
        out.append(line_visual(f"{tag}_bound{j}", x_center, by, x_len, LINE_W))
    return out


def zone_pillar_positions(z):
    """구역 통로 양옆(칸막이선 x × 통로경계 y) 기둥 좌표 리스트."""
    sx = z['stall_x']
    x_div = ([sx[0] - STALL_W / 2 - LINE_W / 2]
             + [(sx[i] + sx[i + 1]) / 2 for i in range(len(sx) - 1)]
             + [sx[-1] + STALL_W / 2 + LINE_W / 2])
    rows = sorted(z['rows'])
    aisle_ys = [rows[-1] - STALL_L / 2, rows[0] + STALL_L / 2]  # 위줄 아래모서리 / 아래줄 위모서리
    return [(x, y) for y in aisle_ys for x in x_div]


def pillar_elems(positions):
    """기둥 collision + visual (회색)."""
    h, half = PILLAR_H, PILLAR_H / 2
    out = []
    for i, (x, y) in enumerate(positions):
        box = f"<box><size>{PILLAR_SIZE} {PILLAR_SIZE} {h}</size></box>"
        out.append(
            f"""      <collision name="pc{i}"><pose>{x:.3f} {y:.3f} {half} 0 0 0</pose>
        <geometry>{box}</geometry></collision>
      <visual name="pv{i}"><pose>{x:.3f} {y:.3f} {half} 0 0 0</pose>
        <geometry>{box}</geometry>
        <material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.5 0.5 0.5 1</diffuse></material>
      </visual>
""")
    return out


def plate_visual(name, px, py, pz, label, yaw=0.0):
    """번호판 얇은 판 visual (CarPlate/<label> 머티리얼)."""
    sx, sy, sz = PLATE_SIZE
    return f"""        <visual name="{name}">
          <pose>{px} {py} {pz} 0 0 {yaw}</pose>
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <material><script>
            <uri>model://target_car/materials/scripts</uri>
            <uri>model://target_car/materials/textures</uri>
            <name>CarPlate/{label}</name>
          </script></material>
        </visual>"""


def car_models(cars):
    """주차 차량 — 각 차량 독립 <model>(static). 메시 visual + 박스 collision + 앞/뒤 번호판.
    메시 front=-Y 가 yaw 로 통로를 향함(위줄 yaw0 / 아래줄 yawπ)."""
    out = []
    for label, (x, y, yaw, color, _plate_num) in cars.items():
        out.append(
            f"""    <model name="car_{label}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} 0 0 0 {yaw}</pose>
      <link name="body">
        <collision name="col"><pose>0 0 {CAR_Z} 0 0 0</pose>
          <geometry><box><size>{MESH_W} {MESH_L} {CAR_H}</size></box></geometry></collision>
        <visual name="vis">
          <geometry><mesh><uri>model://target_car/meshes/Car_{color}.dae</uri><scale>1 1 1</scale></mesh></geometry>
        </visual>
{plate_visual('plate_front', 0, PLATE_FRONT_Y, PLATE_Z, label)}
{plate_visual('plate_rear', 0, PLATE_REAR_Y, PLATE_Z, label)}
      </link>
    </model>
""")
    return out


def dynamic_car_model(dc):
    """동적 차량 1대 — 실제 Car 메시(색). 진짜 물리 바디 + planar_move 로 속도 구동.
    (kinematic 텔레포트는 로봇을 불도저처럼 밀고 정적 장애물을 뚫어 → 물리 바디로 교체)
    → /dyn_car/cmd_vel 로 굴리고 /dyn_car/odom 발행. 충돌 정상, 로봇은 costmap 회피.
    메시 front=-Y 를 모델 +x(planar_move 전진)로 정렬하려 visual·plate yaw +90°,
    충돌박스는 그에 맞춰 길이축 x (MESH_L × MESH_W)."""
    m = 200.0
    # 회전 후: x=길이(MESH_L), y=폭(MESH_W), z=높이
    ixx = m / 12.0 * (MESH_W ** 2 + CAR_H ** 2)
    iyy = m / 12.0 * (MESH_L ** 2 + CAR_H ** 2)
    izz = m / 12.0 * (MESH_L ** 2 + MESH_W ** 2)
    return f"""    <model name="{dc['name']}">
      <pose>{dc['x']:.3f} {dc['y']:.3f} 0 0 0 {dc['yaw']}</pose>
      <link name="body">
        <inertial><pose>0 0 {CAR_Z} 0 0 0</pose>
          <mass>{m}</mass>
          <inertia><ixx>{ixx:.1f}</ixx><iyy>{iyy:.1f}</iyy><izz>{izz:.1f}</izz>
            <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>
        <collision name="col"><pose>0 0 {CAR_Z} 0 0 0</pose>
          <geometry><box><size>{MESH_L} {MESH_W} {CAR_H}</size></box></geometry></collision>
        <visual name="vis"><pose>0 0 0 0 0 {HALF_PI}</pose>
          <geometry><mesh><uri>model://target_car/meshes/Car_{dc['color']}.dae</uri><scale>1 1 1</scale></mesh></geometry>
        </visual>
{plate_visual('plate_front', PLATE_FRONT_Y * -1, 0, PLATE_Z, dc['plate'], HALF_PI)}
      </link>
      <plugin name="dyn_car_drive" filename="libgazebo_ros_planar_move.so">
        <ros><namespace>/dyn_car</namespace></ros>
        <update_rate>50</update_rate>
        <publish_rate>30</publish_rate>
        <publish_odom>true</publish_odom>
        <publish_odom_tf>false</publish_odom_tf>
        <odometry_frame>dyn_car_odom</odometry_frame>
        <robot_base_frame>{dc['name']}</robot_base_frame>
        <covariance_x>0.0</covariance_x>
        <covariance_y>0.0</covariance_y>
        <covariance_yaw>0.0</covariance_yaw>
      </plugin>
    </model>
"""


def landmark_includes(landmarks):
    """비대칭 랜드마크 — Fuel 모델을 static include (네이티브 메시/머티리얼 사용)."""
    out = []
    for name, (model, x, y, yaw, _sx, _sy) in landmarks.items():
        out.append(
            f"""    <include>
      <uri>model://{model}</uri>
      <name>lm_{name}</name>
      <static>true</static>
      <pose>{x:.3f} {y:.3f} 0 0 0 {yaw}</pose>
    </include>
""")
    return out


def main():
    visuals = []
    for tag, z in ZONES.items():
        visuals += zone_lines(tag, z)

    pillars = []
    for tag in PILLAR_ZONES:
        pillars += pillar_elems(zone_pillar_positions(ZONES[tag]))

    cars = car_models(CARS)
    landmarks = landmark_includes(LANDMARKS)
    dyn_car = dynamic_car_model(DYNAMIC_CAR)

    world = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="parking_lot">
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>

    <include>
      <uri>model://Distribution_Warehouse</uri>
      <pose>0 0 0 0 0 0</pose>
    </include>

    <!-- 주차라인: 충돌 없는 흰 visual 묶음 (static) -->
    <model name="parking_lines">
      <static>true</static>
      <link name="lines">
{''.join(visuals)}      </link>
    </model>

    <!-- 기둥: collision+visual (AMCL 특징 + 정적 장애물) -->
    <model name="pillars">
      <static>true</static>
      <link name="pillars_link">
{''.join(pillars)}      </link>
    </model>

    <!-- 주차 차량: OCR 타깃 (collision+visual + 번호판) -->
{''.join(cars)}
    <!-- 비대칭 랜드마크: AMCL aliasing 방지 (map 에도 등록됨) -->
{''.join(landmarks)}
    <!-- 동적 차량: 고정경로 진입(Iter3) — set_entity_state 로 텔레포트 -->
{dyn_car}  </world>
</sdf>
"""
    out_path = os.path.join(os.path.dirname(__file__), '..', 'worlds', 'parking_lot.world')
    with open(out_path, 'w') as f:
        f.write(world)
    print(f"{out_path} 생성 완료 — 라인 {len(visuals)}개 ({len(ZONES)}구역 × 18), "
          f"기둥 {len(pillars)}개 ({PILLAR_ZONES}), 차량 {len(cars)}대 ({list(CARS)}), "
          f"랜드마크 {len(landmarks)}개 ({list(LANDMARKS)}), "
          f"동적차량 1대 ({DYNAMIC_CAR['name']} @ {DYNAMIC_CAR['x']},{DYNAMIC_CAR['y']})")


if __name__ == '__main__':
    main()
