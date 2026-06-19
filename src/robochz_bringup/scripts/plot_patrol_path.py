#!/usr/bin/env python3
"""주행경로 PNG — A구역 슬롯 레이아웃(스키매틱) 위에 주행궤적/차량/waypoint 오버레이.

path_recorder/monitor 가 저장한 CSV(log/robochz_captures/patrol_path.csv)를 읽어
사용자 제공 A-zone 슬롯 도면 위에 렌더 → PNG. matplotlib+pyyaml 필요(OCR venv 에 있음):
    <venv>/bin/python scripts/plot_patrol_path.py
출력: log/robochz_captures/patrol_path.png
"""
import os
import re
import csv
import json
import math
import argparse
import yaml
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.font_manager as fm
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.collections import LineCollection

# 한글 번호판 렌더용 폰트 (Noto Sans CJK)
for _name in ('Noto Sans CJK KR', 'Noto Sans CJK JP', 'NanumGothic', 'Noto Sans CJK HK'):
    if any(f.name == _name for f in fm.fontManager.ttflist):
        matplotlib.rcParams['font.family'] = _name
        break
matplotlib.rcParams['axes.unicode_minus'] = False

from gen_parking_world import CARS, REGISTERED   # 차량 좌표/등록여부 오버레이 (같은 폴더)

HERE = os.path.dirname(__file__)
WP_YAML = os.path.join(HERE, '..', 'config', 'patrol_waypoints.yaml')
LOG = os.path.expanduser('~/workspace/ros2_ws/robochz_ws/log/robochz_captures')
CSV_IN = os.path.join(LOG, 'patrol_path.csv')
PNG_OUT = os.path.join(LOG, 'patrol_path.png')

REG = REGISTERED                     # 등록 차량(파랑) / 그 외 미등록(빨강)
CAR_W, CAR_L = 1.9, 4.7              # 차량 footprint (폭, 깊이)

# ── A구역 슬롯 레이아웃 파라미터 (사용자 제공 도면) ──
X0, X1 = 4.0, 19.7
Y0, Y1 = 0.0, 16.0
SLOT_W = 2.5      # 주차칸 폭
SLOT_H = 5.0      # 주차칸 깊이
LINE_W = 0.1      # 주차선 폭
AISLE_H = 6.0     # 통로 폭
TOP_Y, BOTTOM_Y = 13.5, 2.5
X_CENTERS = [X0 + LINE_W + SLOT_W / 2 + i * (SLOT_W + LINE_W) for i in range(6)]


def draw_zone(ax):
    """사용자 제공 A-zone 슬롯 도면(외곽·통로·주차칸·라벨·코너좌표)."""
    # A구역 외곽
    ax.add_patch(patches.Rectangle((X0, Y0), X1 - X0, Y1 - Y0, fill=False, linewidth=2))
    # 통로
    ax.add_patch(patches.Rectangle((X0, 5.0), X1 - X0, AISLE_H, alpha=0.10))
    ax.text((X0 + X1) / 2, 8.0, '6m aisle', ha='center', va='center', fontsize=12)
    # 위/아래 주차 영역
    ax.add_patch(patches.Rectangle((X0, 11.0), X1 - X0, SLOT_H, fill=False, linewidth=1.5))
    ax.add_patch(patches.Rectangle((X0, 0.0), X1 - X0, SLOT_H, fill=False, linewidth=1.5))
    # 주차칸 + 라벨/좌표
    for i, xc in enumerate(X_CENTERS):
        sx = X0 + LINE_W + i * (SLOT_W + LINE_W)
        ax.add_patch(patches.Rectangle((sx, 11.0), SLOT_W, SLOT_H, fill=False, linewidth=1))
        ax.add_patch(patches.Rectangle((sx, 0.0), SLOT_W, SLOT_H, fill=False, linewidth=1))
        ax.text(xc, TOP_Y, f'A-{i + 1:02d}', ha='center', va='center',
                fontsize=11, fontweight='bold', zorder=6)
        ax.text(xc, TOP_Y - 0.7, f'({xc:.2f},{TOP_Y:.1f})', ha='center', va='center',
                fontsize=7, zorder=6)
        ax.text(xc, BOTTOM_Y, f'A-{i + 7:02d}', ha='center', va='center',
                fontsize=11, fontweight='bold', zorder=6)
        ax.text(xc, BOTTOM_Y - 0.7, f'({xc:.2f},{BOTTOM_Y:.1f})', ha='center', va='center',
                fontsize=7, zorder=6)
    # 코너 좌표
    for x, y in [(X0, Y1), (X1, Y1), (X0, Y0), (X1, Y0)]:
        ax.scatter(x, y, s=35, color='k', zorder=6)
        off = 0.35 if y == Y1 else -0.35
        va = 'bottom' if y == Y1 else 'top'
        ax.text(x, y + off, f'({x:.1f},{y:.1f})', ha='center', va=va, fontsize=9)


def overlay_detections(ax, det_path):
    """monitor 가 저장한 detections.json 의 OCR 결과를 각 주차칸에 오버레이.
    등록=초록 / 미등록=빨강 / 번호판없음=회색."""
    if not os.path.exists(det_path):
        return
    try:
        dets = json.load(open(det_path, encoding='utf-8'))
    except Exception:
        return
    for d in dets:
        m = re.match(r'A-?0*(\d+)', d.get('waypoint_id', ''))
        if not m:
            continue
        n = int(m.group(1))
        if 1 <= n <= 6:
            xc, ty = X_CENTERS[n - 1], TOP_Y + 1.95       # 위줄: 슬롯 상단
        elif 7 <= n <= 12:
            xc, ty = X_CENTERS[n - 7], BOTTOM_Y - 1.95    # 아래줄: 슬롯 하단
        else:
            continue
        plate = (d.get('plate_text') or '').strip()
        if d.get('success') and plate:
            txt = plate
            fc = 'tab:green' if d.get('is_registered') else 'tab:red'
        else:
            txt, fc = '(no plate)', '0.5'
        ax.text(xc, ty, txt, ha='center', va='center', fontsize=8.5, fontweight='bold',
                color='white', zorder=8,
                bbox=dict(boxstyle='round,pad=0.25', fc=fc, ec='none', alpha=0.92))


def main():
    ap = argparse.ArgumentParser(description='A-zone patrol path PNG')
    ap.add_argument('--csv', default=CSV_IN, help='주행궤적 CSV 경로')
    ap.add_argument('--png', default=PNG_OUT, help='출력 PNG 경로')
    args = ap.parse_args()
    csv_in, png_out = args.csv, args.png

    fig, ax = plt.subplots(figsize=(11, 7))
    draw_zone(ax)

    # 차량 오버레이 (등록=파랑 / 미등록=빨강). zorder 낮춰 라벨이 위로.
    for lbl, (x, y, yaw, color, plate) in CARS.items():
        col = 'tab:blue' if lbl in REG else 'tab:red'
        ax.add_patch(patches.Rectangle((x - CAR_W / 2, y - CAR_L / 2), CAR_W, CAR_L,
                                       color=col, alpha=0.40, zorder=2))

    # 인식결과(번호판) 오버레이 — csv 와 같은 폴더의 detections.json (등록=초록/미등록=빨강)
    overlay_detections(ax, os.path.join(os.path.dirname(os.path.abspath(csv_in)),
                                        'detections.json'))

    # waypoint
    wp = yaml.safe_load(open(WP_YAML))['patrol_node']['ros__parameters']
    ax.plot(wp['waypoints_x'], wp['waypoints_y'], 'k+', ms=10, zorder=4, label='waypoints')

    # 주행 궤적 (CSV: 'x,y' 또는 't,x,y')
    if os.path.exists(csv_in):
        rows = list(csv.reader(open(csv_in)))
        header = [c.strip().lower() for c in rows[0]]
        data = rows[1:]
        has_t = header[:1] == ['t']            # 시간 컬럼 유무
        if has_t:
            ts = [float(r[0]) for r in data]
            px = [float(r[1]) for r in data]
            py = [float(r[2]) for r in data]
        else:
            ts = None
            px = [float(r[0]) for r in data]
            py = [float(r[1]) for r in data]

        if has_t and len(px) >= 2:
            # 구간별 속도(거리/Δt) → 저속(지연)=빨강, 정속=초록 으로 강조
            pts = np.column_stack([px, py])
            segs = np.stack([pts[:-1], pts[1:]], axis=1)
            spd = []
            for i in range(len(px) - 1):
                d = math.hypot(px[i + 1] - px[i], py[i + 1] - py[i])
                dt = ts[i + 1] - ts[i]
                spd.append(d / dt if dt > 1e-6 else 0.0)
            spd = np.array(spd)
            lc = LineCollection(segs, cmap='RdYlGn', zorder=5, linewidths=3.0)
            lc.set_array(spd)
            lc.set_clim(0.0, max(spd.max(), 0.05))   # 0=정차(빨강) ~ 최대속도(초록)
            ax.add_collection(lc)
            cb = fig.colorbar(lc, ax=ax, fraction=0.035, pad=0.02)
            cb.set_label('speed [m/s]  (red = delayed / slow)')
            # 지연(정차) 구간 강조: Δt 큰 지점에 머문시간 비례 빨간 링
            dt_arr = np.array([ts[i + 1] - ts[i] for i in range(len(px) - 1)])
            dwell = np.where(dt_arr > 1.0)[0]      # 1초 이상 머문 구간
            if len(dwell):
                ax.scatter([px[i + 1] for i in dwell], [py[i + 1] for i in dwell],
                           s=[60 + 50 * dt_arr[i] for i in dwell],
                           facecolors='none', edgecolors='red', linewidths=2.0,
                           zorder=8, label='delay (dwell)')
            # 총 소요시간 우측하단
            total = ts[-1] - ts[0]
            ax.text(0.98, 0.02, f'total time: {total:.1f} s',
                    transform=ax.transAxes, ha='right', va='bottom', fontsize=12,
                    bbox=dict(boxstyle='round', fc='white', ec='0.5', alpha=0.9), zorder=8)
            print(f'경로점 {len(px)}개 · 총 {total:.1f}s 로드 (시간색상)')
        else:
            ax.plot(px, py, '-', color='orange', lw=2.5, zorder=5, label='driven path')
            print(f'경로점 {len(px)}개 로드 (시간정보 없음 → 단색)')

        ax.plot(px[0], py[0], 'go', ms=11, zorder=7, label='start')
        ax.plot(px[-1], py[-1], 'rs', ms=11, zorder=7, label='end')
    else:
        print(f'[!] {csv_in} 없음 — 순찰 한 번 돌려 path_recorder 가 저장하게 하세요')

    ax.set_title('Cheezlbot A-zone patrol path', fontsize=14)
    ax.text((X0 + X1) / 2, Y1 + 0.7, 'A Zone: 6 x 2 parking slots',
            ha='center', va='bottom', fontsize=13)
    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
    ax.set_xlim(2.5, 21.2); ax.set_ylim(-1.2, 17.8)
    ax.set_aspect('equal', adjustable='box')
    ax.grid(True, linewidth=0.5)
    # 자동 핸들(waypoints/path/start/end) + 차량색 프록시 합쳐 범례
    handles, _ = ax.get_legend_handles_labels()
    handles += [patches.Patch(color='tab:blue', alpha=0.40, label='registered car'),
                patches.Patch(color='tab:red', alpha=0.40, label='unregistered car')]
    ax.legend(handles=handles, loc='upper right', fontsize=9)
    fig.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(png_out)), exist_ok=True)
    fig.savefig(png_out, dpi=200)
    print(f'저장: {png_out}')


if __name__ == '__main__':
    main()
