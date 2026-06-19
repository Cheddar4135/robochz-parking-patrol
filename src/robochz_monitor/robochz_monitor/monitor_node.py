import os
import csv
import json
import shutil
import subprocess

# 로그 줄 포맷: "[INFO] [ts] [node]: msg" → 메시지만 (rclpy 로깅 초기화 전 설정)
os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{message}'

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Bool
from nav_msgs.msg import Path
from robochz_msgs.msg import PlateDetection


def latched_qos():
    """patrol 의 /patrol_status(터미널 1회 신호) 와 호환되는 latched 프로파일."""
    qos = QoSProfile(depth=1)
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    qos.reliability = ReliabilityPolicy.RELIABLE
    return qos


class MonitorNode(Node):                          # rclpy.node.Node 상속
    def __init__(self):
        super().__init__('monitor_node')
        self.get_logger().info('monitor_node 시작')

        # ── 경로 파라미터 (머신 종속이라 노출, 기본은 expanduser) ──
        home = os.path.expanduser('~/workspace/ros2_ws/robochz_ws')
        self.declare_parameter('captures_dir', os.path.join(home, 'log', 'robochz_captures'))
        self.declare_parameter('report_root', os.path.join(home, 'log', 'reports'))
        self.declare_parameter(
            'plot_script', os.path.join(home, 'src', 'robochz_bringup', 'scripts',
                                        'plot_patrol_path.py'))
        # plot 스크립트는 matplotlib 필요 → OCR venv 파이썬 재사용
        self.declare_parameter(
            'plot_python',
            os.path.expanduser('~/workspace/ros2_ws/korean_ocr_using_paddleOCR/.venv/bin/python'))
        self._captures_dir = self.get_parameter('captures_dir').value
        self._report_root = self.get_parameter('report_root').value
        self._plot_script = self.get_parameter('plot_script').value
        self._plot_python = self.get_parameter('plot_python').value

        # 등록 차량 DB (perception 과 동일 파일) — 시작 시 목록 출력
        try:
            default_db = os.path.join(
                get_package_share_directory('robochz_perception'),
                'config', 'registered_vehicles.json')
        except Exception:
            default_db = os.path.join(home, 'src', 'robochz_perception',
                                      'config', 'registered_vehicles.json')
        self.declare_parameter('registered_db', default_db)
        self._print_registered(self.get_parameter('registered_db').value)

        # ── 상태 ──
        self._latest_path = None        # 최신 /patrol_path (완주 시 CSV 로 굳힘)
        self._report_done = False       # 한 세션 1회만 스냅샷(latched 재수신 방지)
        self._detections = {}           # waypoint_id → 인식결과 (리포트 png 오버레이용)

        # ── 구독 ──
        # 1) 인식 결과 스트림 (기존 로깅 유지)
        self.create_subscription(
            PlateDetection, '/plate_detection', self.plate_detection_callback, 10)
        # 2) 주행 경로 (path_recorder 가 누적 발행 → 최신본 보관)
        self.create_subscription(Path, '/patrol_path', self._on_path, 10)
        # 3) 완주 신호 (latched) → 리포트 스냅샷 트리거
        self.create_subscription(
            Bool, '/patrol_status', self._on_status, latched_qos())

    # ---------- 등록 차량 목록 ----------
    def _print_registered(self, path):
        try:
            with open(path, encoding='utf-8') as f:
                reg = json.load(f).get('registered', [])
        except Exception as e:
            self.get_logger().warn(f'등록 DB 로드 실패({path}): {e}')
            return
        self.get_logger().info(f'━━━━ 등록 차량 {len(reg)}대 ━━━━')
        for e in reg:
            self.get_logger().info(
                f"  · {e.get('plate', '?')}  ({e.get('stall', '?')}, {e.get('owner', '')})")
        self.get_logger().info('━━━━━━━━━━━━━━━━━━')

    # ---------- 인식 결과 ----------
    def plate_detection_callback(self, msg: PlateDetection):
        status = '등록' if msg.is_registered else '미등록'
        self.get_logger().info(
            f'[번호판 인식] '
            f'순찰지점={msg.waypoint_id}, '
            f'번호판={msg.plate_text}, '
            f'상태={status}'
        )
        # 리포트 png 오버레이용 누적 (waypoint_id 당 최신)
        self._detections[msg.waypoint_id] = {
            'waypoint_id': msg.waypoint_id,
            'plate_text': msg.plate_text,
            'is_registered': bool(msg.is_registered),
            'success': bool(msg.success),
            'confidence': float(msg.confidence),
        }

    # ---------- 주행 경로 ----------
    def _on_path(self, msg: Path):
        self._latest_path = msg

    # ---------- 완주 → 리포트 ----------
    def _on_status(self, msg: Bool):
        if not msg.data or self._report_done:
            return
        self._report_done = True
        try:
            self._make_report()
        except Exception as e:                     # 리포트 실패가 노드를 죽이지 않게
            self.get_logger().error(f'리포트 생성 실패: {e}')

    def _next_report_dir(self):
        """log/reports/reports1, reports2 … 중 비어있는 다음 번호를 골라 생성."""
        os.makedirs(self._report_root, exist_ok=True)
        n = 1
        while True:
            d = os.path.join(self._report_root, f'reports{n}')
            if not os.path.exists(d):
                os.makedirs(d)
                return d
            n += 1

    def _freeze_path_csv(self):
        """최신 /patrol_path 를 captures_dir/patrol_path.csv 로 굳힘(이 런 경로 반영).
        path_recorder 는 shutdown 시에만 저장하므로, 완주 시점 png 를 위해 여기서 갱신."""
        if self._latest_path is None or not self._latest_path.poses:
            self.get_logger().warn('/patrol_path 수신 없음 → CSV 갱신 생략(기존 유지)')
            return
        os.makedirs(self._captures_dir, exist_ok=True)
        csv_path = os.path.join(self._captures_dir, 'patrol_path.csv')
        poses = self._latest_path.poses
        t0 = poses[0].header.stamp.sec + poses[0].header.stamp.nanosec * 1e-9
        with open(csv_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'x', 'y'])      # t = 첫 점 기준 상대시각(초)
            for ps in poses:
                t = ps.header.stamp.sec + ps.header.stamp.nanosec * 1e-9 - t0
                w.writerow([f'{t:.3f}', f'{ps.pose.position.x:.4f}',
                            f'{ps.pose.position.y:.4f}'])
        self.get_logger().info(
            f'주행경로 {len(self._latest_path.poses)}점 → {csv_path}')

    def _render_path_png(self):
        """venv 파이썬으로 plot_patrol_path.py 실행 → captures_dir/patrol_path.png 재생성."""
        if not (os.path.exists(self._plot_python) and os.path.exists(self._plot_script)):
            self.get_logger().warn(
                f'plot 도구 없음(py={self._plot_python}, script={self._plot_script}) → png 생략')
            return
        try:
            r = subprocess.run(
                [self._plot_python, self._plot_script],
                cwd=os.path.dirname(self._plot_script),
                capture_output=True, text=True, timeout=120)
            if r.returncode != 0:
                self.get_logger().warn(f'plot 실패(rc={r.returncode}): {r.stderr.strip()[:200]}')
            else:
                self.get_logger().info('경로 시각화 png 재생성 완료')
        except Exception as e:
            self.get_logger().warn(f'plot 실행 오류: {e}')

    def _write_detections(self):
        """누적 인식결과를 captures_dir/detections.json 으로 저장(plot 이 png 에 오버레이)."""
        os.makedirs(self._captures_dir, exist_ok=True)
        path = os.path.join(self._captures_dir, 'detections.json')
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(list(self._detections.values()), f, ensure_ascii=False, indent=2)
        self.get_logger().info(f'인식결과 {len(self._detections)}건 → {path}')

    def _make_report(self):
        report_dir = self._next_report_dir()

        # 1) 경로 CSV + 인식결과 JSON 굳히고 → 2) png 재생성(번호판 오버레이 포함)
        self._freeze_path_csv()
        self._write_detections()
        self._render_path_png()

        # 3) robochz_captures 통째 복사 (캡처 이미지 + csv + png + 워커로그)
        dst_captures = os.path.join(report_dir, 'robochz_captures')
        if os.path.isdir(self._captures_dir):
            shutil.copytree(self._captures_dir, dst_captures)
        else:
            self.get_logger().warn(f'captures_dir 없음: {self._captures_dir}')

        # 4) full 경로 시각화 png 를 리포트 최상단에도 배치
        src_png = os.path.join(self._captures_dir, 'patrol_path.png')
        if os.path.exists(src_png):
            shutil.copy2(src_png, os.path.join(report_dir, 'patrol_path.png'))

        self.get_logger().info(f'★ 순찰 리포트 생성: {report_dir}')


def main(args=None):
    rclpy.init(args=args)                         # ROS2 통신 시스템 초기화
    node = MonitorNode()
    try:
        rclpy.spin(node)                          # 콜백 받을 때까지 대기
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('monitor_node 종료')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
