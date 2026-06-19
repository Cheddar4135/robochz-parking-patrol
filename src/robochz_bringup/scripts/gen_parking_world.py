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

# ── 주차 차량 (Iter2 OCR 타깃) ──
# 박스 차체 + 앞면(local +x)에 번호판 visual. yaw로 회전해 번호판이 통로를 향함.
# 번호판 텍스처/머티리얼은 models/target_car/materials 에 있음 (CarPlate/<라벨>).
CAR_SIZE = (4.7, 1.9, 1.5)        # 차체 박스 (길이 x, 폭 y, 높이 z) — 회전 전 local 기준
CAR_Z = CAR_SIZE[2] / 2           # 차체 중심 높이(0.75) → 바닥에 앉음
PLATE_SIZE = (0.01, 0.335, 0.17)  # 번호판 얇은 판 (두께, 폭, 높이)
PLATE_LOCAL = (2.36, 0.0, -0.25)  # body 링크중심 기준 번호판 위치 (앞면 1cm 돌출 / 월드z 0.50)
HALF_PI = 1.5708                  # 90° (rad)

# 라벨: (중심 x, 중심 y, yaw, 머티리얼)
#   위줄(y=13.5)=A-01~06 → 번호판이 통로(-y) 향하게 yaw=-90°
#   아래줄(y=2.5)=A-07~12 → 번호판이 통로(+y) 향하게 yaw=+90°
CARS = {
    'A01': (5.35,  13.5, -HALF_PI, 'CarPlate/A01'),
    'A02': (7.95,  13.5, -HALF_PI, 'CarPlate/A02'),
    'A05': (15.75, 13.5, -HALF_PI, 'CarPlate/A05'),
    'A08': (7.95,  2.5,   HALF_PI, 'CarPlate/A08'),
    'A12': (18.35, 2.5,   HALF_PI, 'CarPlate/A12'),
}

# ── 비대칭 랜드마크 (AMCL aliasing 방지) ──
# 통로가 x축으로 반복구조라 위치추정이 통로 따라 미끄러짐(aliasing) → 빈 주차칸 입구/끝에
# 서로 다른 사물을 비대칭 배치해 각 지점의 특징을 유일하게 만든다.
# 차량과 달리 영구 고정물이므로 map 에도 등록(gen_map_from_world 가 import).
#   name: (model_uri, x, y, yaw, map_sx, map_sy)  — map_sx/sy = 맵 래스터용 박스(회전 전 footprint)
LANDMARKS = {
    'barrier':  ('drc_practice_orange_jersey_barrier', 4.4,   8.0,  HALF_PI, 1.6, 0.6),  # 좌측 벽, 진입부(중앙 안 막음)
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


def car_models(cars):
    """주차 차량들 — 각 차량을 독립 <model>(static)로. 차체 collision+visual + 번호판 visual."""
    cw, cl, ch = CAR_SIZE                       # 회전 전 local: 길이x, 폭y, 높이z
    pt, pw, ph = PLATE_SIZE
    px, py, pz = PLATE_LOCAL
    out = []
    for label, (x, y, yaw, mat) in cars.items():
        out.append(
            f"""    <model name="car_{label}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} 0 0 0 {yaw}</pose>
      <link name="body">
        <pose>0 0 {CAR_Z} 0 0 0</pose>
        <collision name="col"><geometry><box><size>{cw} {cl} {ch}</size></box></geometry></collision>
        <visual name="body_vis">
          <geometry><box><size>{cw} {cl} {ch}</size></box></geometry>
          <material><ambient>0.25 0.25 0.30 1</ambient><diffuse>0.30 0.30 0.42 1</diffuse></material>
        </visual>
        <visual name="plate">
          <pose>{px} {py} {pz} 0 0 0</pose>
          <geometry><box><size>{pt} {pw} {ph}</size></box></geometry>
          <material><script>
            <uri>model://target_car/materials/scripts</uri>
            <uri>model://target_car/materials/textures</uri>
            <name>{mat}</name>
          </script></material>
        </visual>
      </link>
    </model>
""")
    return out


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
{''.join(landmarks)}  </world>
</sdf>
"""
    out_path = os.path.join(os.path.dirname(__file__), '..', 'worlds', 'parking_lot.world')
    with open(out_path, 'w') as f:
        f.write(world)
    print(f"{out_path} 생성 완료 — 라인 {len(visuals)}개 ({len(ZONES)}구역 × 18), "
          f"기둥 {len(pillars)}개 ({PILLAR_ZONES}), 차량 {len(cars)}대 ({list(CARS)}), "
          f"랜드마크 {len(landmarks)}개 ({list(LANDMARKS)})")


if __name__ == '__main__':
    main()
