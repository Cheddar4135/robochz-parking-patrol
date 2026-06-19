#!/usr/bin/env python3
"""주행경로 PNG 저장 — 맵 + 궤적 + waypoint + 차량 오버레이 (보고서용).

path_recorder 가 저장한 CSV(log/robochz_captures/patrol_path.csv)와 맵을 읽어
matplotlib 으로 렌더 → PNG. matplotlib 필요(OCR venv 에 있음):
    <venv>/bin/python scripts/plot_patrol_path.py
출력: log/robochz_captures/patrol_path.png
"""
import os
import csv
import yaml
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from gen_parking_world import CARS, LANDMARKS   # 오버레이용 (같은 폴더)

HERE = os.path.dirname(__file__)
MAP_YAML = os.path.join(HERE, '..', 'maps', 'parking_lot.yaml')
WP_YAML = os.path.join(HERE, '..', 'config', 'patrol_waypoints.yaml')
LOG = os.path.expanduser('~/workspace/ros2_ws/robochz_ws/log/robochz_captures')
CSV_IN = os.path.join(LOG, 'patrol_path.csv')
PNG_OUT = os.path.join(LOG, 'patrol_path.png')


def load_pgm(path):
    with open(path, 'rb') as f:
        assert f.readline().strip() == b'P5'
        w, h = map(int, f.readline().split())
        f.readline()
        data = np.frombuffer(f.read(), np.uint8).reshape(h, w)
    return data


def main():
    m = yaml.safe_load(open(MAP_YAML))
    res, (ox, oy, _) = m['resolution'], m['origin']
    img = load_pgm(os.path.join(HERE, '..', 'maps', m['image']))
    h, w = img.shape
    extent = [ox, ox + w * res, oy, oy + h * res]

    fig, ax = plt.subplots(figsize=(11, 8))
    ax.imshow(img, cmap='gray', extent=extent, origin='upper', zorder=0)

    # 차량 (등록/미등록 색)
    reg = {'A01', 'A02', 'A08'}
    for lbl, (x, y, yaw, mat) in CARS.items():
        col = 'tab:blue' if lbl in reg else 'tab:red'
        ax.add_patch(plt.Rectangle((x - 0.95, y - 2.35), 1.9, 4.7, color=col,
                                   alpha=0.35, zorder=1))
        ax.text(x, y, lbl, ha='center', va='center', fontsize=8, zorder=3)

    # 랜드마크
    for lbl, (mdl, x, y, yaw, sx, sy) in LANDMARKS.items():
        ax.plot(x, y, 'g^', ms=8, zorder=3)

    # waypoint
    wp = yaml.safe_load(open(WP_YAML))['patrol_node']['ros__parameters']
    ax.plot(wp['waypoints_x'], wp['waypoints_y'], 'k+', ms=10, zorder=4,
            label='waypoints')

    # 주행 궤적
    if os.path.exists(CSV_IN):
        rows = list(csv.reader(open(CSV_IN)))[1:]
        px = [float(r[0]) for r in rows]
        py = [float(r[1]) for r in rows]
        ax.plot(px, py, '-', color='orange', lw=2.5, zorder=2, label='driven path')
        ax.plot(px[0], py[0], 'go', ms=10, zorder=5, label='start')
        ax.plot(px[-1], py[-1], 'rs', ms=10, zorder=5, label='end')
        print(f'경로점 {len(px)}개 로드')
    else:
        print(f'[!] {CSV_IN} 없음 — 순찰 한 번 돌려 path_recorder 가 저장하게 하세요')

    ax.set_xlim(2, 21); ax.set_ylim(-1, 17)
    ax.set_aspect('equal'); ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
    ax.set_title('Cheezlbot A-zone patrol path')
    ax.legend(loc='upper right', fontsize=9)
    fig.tight_layout()
    fig.savefig(PNG_OUT, dpi=130)
    print(f'저장: {PNG_OUT}')


if __name__ == '__main__':
    main()
