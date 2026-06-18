#!/usr/bin/env python3
"""Ground-truth occupancy grid 생성기 (SLAM 대체).

우리가 만든 월드라 기하를 알고 있으므로, Distribution_Warehouse 의 collision 박스를
로봇 LiDAR 높이에서 2D 래스터화해 점유격자(.pgm + .yaml)를 직접 만든다.

  - LiDAR 높이밴드에 걸치는 박스만 장애물로 (천장/바닥 박스 제외)
  - spawn 에서 flood-fill 한 영역만 free, 벽 밖은 unknown
  - 주차라인(parking_lines)은 collision 이 없어 자동으로 제외됨

출력: ../maps/parking_lot.{pgm,yaml}
"""
import os
import math
import struct
import collections
import xml.etree.ElementTree as ET

HERE = os.path.dirname(__file__)
MODEL_SDF = os.path.join(HERE, '..', 'models', 'Distribution_Warehouse', 'model.sdf')
WORLD = os.path.join(HERE, '..', 'worlds', 'parking_lot.world')   # 인라인 모델(기둥) collision 포함
OUT_BASE = os.path.join(HERE, '..', 'maps', 'parking_lot')

RES = 0.05                 # m/pixel
H_LO, H_HI = 0.05, 0.30    # 로봇 LiDAR 높이밴드 — 이 구간에 걸치는 박스만 장애물
SPAWN = (11.86, 8.0)       # flood-fill 시작점 (로봇 spawn = A구역 통로)
# 맵 crop = A구역(x4~19.7, y0~16) + 여유 2.5m. 빈 창고 전체 대신 순찰 영역만
# → costmap 축소 + 특징 밀집. (확장 시 이 범위를 넓히면 됨)
CROP_X = (1.5, 22.2)
CROP_Y = (-2.5, 18.5)

FREE, OCC, UNK = 254, 0, 205


def load_obstacle_boxes(path):
    """파일에서 LiDAR 높이밴드에 걸치는 (px,py,sx,sy,yaw) 박스 리스트.
    모델/링크가 모두 world 원점(pose 0)에 있어 collision pose 를 곧 world 좌표로 본다."""
    root = ET.parse(path).getroot()
    boxes = []
    for c in root.findall('.//collision'):
        p, s = c.find('pose'), c.find('.//box/size')
        if p is None or s is None:
            continue
        px, py, pz, _r, _p, yaw = map(float, p.text.split())
        sx, sy, sz = map(float, s.text.split())
        if pz - sz / 2 <= H_HI and pz + sz / 2 >= H_LO:   # 높이밴드와 겹침 = 장애물
            boxes.append((px, py, sx, sy, yaw))
    return boxes


def main():
    boxes = load_obstacle_boxes(MODEL_SDF) + load_obstacle_boxes(WORLD)  # 창고 + 인라인 기둥

    # 맵 경계 = crop (순찰 영역만)
    x_min, x_max = CROP_X
    y_min, y_max = CROP_Y
    W = int(math.ceil((x_max - x_min) / RES))
    H = int(math.ceil((y_max - y_min) / RES))

    grid = bytearray([UNK]) * (W * H)

    def cell(wx, wy):
        col = int((wx - x_min) / RES)
        row = int((y_max - wy) / RES)   # pgm 0행 = 최대 y (위)
        return col, row

    # 장애물 스탬프
    for (px, py, sx, sy, yaw) in boxes:
        hx, hy = sx / 2, sy / 2
        corners = []
        c_, s_ = math.cos(yaw), math.sin(yaw)
        for cx, cy in ((hx, hy), (hx, -hy), (-hx, hy), (-hx, -hy)):
            corners.append((px + cx * c_ - cy * s_, py + cx * s_ + cy * c_))
        cxs = [c[0] for c in corners]; cys = [c[1] for c in corners]
        col0, row1 = cell(min(cxs), min(cys))
        col1, row0 = cell(max(cxs), max(cys))
        axis_aligned = abs(yaw) < 1e-3
        for row in range(max(row0, 0), min(row1 + 1, H)):
            for col in range(max(col0, 0), min(col1 + 1, W)):
                if axis_aligned:
                    grid[row * W + col] = OCC
                else:
                    wx = x_min + (col + 0.5) * RES
                    wy = y_max - (row + 0.5) * RES
                    dx, dy = wx - px, wy - py
                    lx = dx * c_ + dy * s_
                    ly = -dx * s_ + dy * c_
                    if abs(lx) <= hx and abs(ly) <= hy:
                        grid[row * W + col] = OCC

    # spawn 에서 flood-fill → 도달 가능한 빈칸만 free
    sc, sr = cell(*SPAWN)
    if not (0 <= sc < W and 0 <= sr < H) or grid[sr * W + sc] == OCC:
        raise SystemExit(f"spawn {SPAWN} 가 맵 밖이거나 장애물 위 (cell {sc},{sr})")
    dq = collections.deque([(sc, sr)])
    grid[sr * W + sc] = FREE
    while dq:
        col, row = dq.popleft()
        for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nc, nr = col + dc, row + dr
            if 0 <= nc < W and 0 <= nr < H and grid[nr * W + nc] == UNK:
                grid[nr * W + nc] = FREE
                dq.append((nc, nr))

    # .pgm (P5 binary)
    with open(OUT_BASE + '.pgm', 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (W, H))
        f.write(bytes(grid))
    # .yaml
    with open(OUT_BASE + '.yaml', 'w') as f:
        f.write(f"image: parking_lot.pgm\nmode: trinary\nresolution: {RES}\n"
                f"origin: [{x_min:.3f}, {y_min:.3f}, 0]\n"
                f"negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")

    occ = sum(1 for v in grid if v == OCC)
    free = sum(1 for v in grid if v == FREE)
    print(f"맵 생성: {W}x{H} @ {RES}m  origin=({x_min:.1f},{y_min:.1f})  "
          f"장애물박스 {len(boxes)}개 / occ {occ} / free {free} 셀")


if __name__ == '__main__':
    main()
