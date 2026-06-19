import os
import json
import subprocess

# 로그 줄 포맷: "[INFO] [ts] [node]: msg" → 메시지만 (rclpy 로깅 초기화 전 설정)
os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{message}'

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.qos import qos_profile_sensor_data

from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image
from robochz_msgs.action import RecognizePlate
from robochz_msgs.msg import PlateDetection


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        self.get_logger().info('perception_node 시작')

        # ── 파라미터 ──
        # OCR venv 파이썬 (paddleocr 설치된 인터프리터). 머신 종속이라 파라미터로 노출.
        default_py = os.path.expanduser(
            '~/workspace/ros2_ws/korean_ocr_using_paddleOCR/.venv/bin/python')
        self.declare_parameter('ocr_python', default_py)
        self.declare_parameter(
            'capture_dir',
            os.path.expanduser('~/workspace/ros2_ws/robochz_ws/log/robochz_captures'))
        # 등록 차량 DB (기본: 패키지 share/config)
        default_db = os.path.join(
            get_package_share_directory('robochz_perception'),
            'config', 'registered_vehicles.json')
        self.declare_parameter('registered_db', default_db)

        ocr_python = self.get_parameter('ocr_python').value
        self._capture_dir = self.get_parameter('capture_dir').value
        os.makedirs(self._capture_dir, exist_ok=True)

        # ── 등록 차량 집합 로드 ──
        self._registered = self._load_db(self.get_parameter('registered_db').value)
        self.get_logger().info(
            f'등록 차량 {len(self._registered)}대 로드: {sorted(self._registered)}')

        # ── 카메라 구독 (최신 프레임만 보관) ──
        # Gazebo 카메라는 Best Effort 가능 → sensor_data QoS 로 호환성 확보
        self._latest = None
        self.create_subscription(
            Image, '/camera/image_raw', self._on_image, qos_profile_sensor_data)

        # ── OCR 워커 1회 기동 (모델 1회 로드 후 상주) ──
        self._worker = self._start_worker(ocr_python)

        # ── 액션 서버 + 결과 토픽 ──
        self._action_server = ActionServer(
            self, RecognizePlate, 'recognize_plate', self.execute_callback)
        self._plate_publisher = self.create_publisher(
            PlateDetection, '/plate_detection', 10)

    # ---------- 초기화 헬퍼 ----------
    def _load_db(self, path):
        try:
            with open(path, encoding='utf-8') as f:
                data = json.load(f)
            return {e['plate'] for e in data.get('registered', [])}
        except Exception as e:
            self.get_logger().warn(f'등록 DB 로드 실패({path}): {e} → 전부 미등록 처리')
            return set()

    def _start_worker(self, ocr_python):
        worker_py = os.path.join(os.path.dirname(__file__), 'ocr_worker.py')
        # paddle 로그/에러는 stdout(프로토콜) 대신 로그파일로 → 기동 실패 시 진단용
        err_log = os.path.join(self._capture_dir, 'ocr_worker.err')
        self.get_logger().info(
            f'OCR 워커 기동: {ocr_python} {worker_py} (모델 로드 대기, 에러로그: {err_log})')
        proc = subprocess.Popen(
            [ocr_python, worker_py],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=open(err_log, 'w'),
            text=True, bufsize=1)
        # READY 올 때까지 대기 (모델 로드 ~수초)
        for line in proc.stdout:
            if line.strip() == 'READY':
                self.get_logger().info('OCR 워커 준비 완료')
                return proc
        raise RuntimeError(f'OCR 워커가 READY 전에 종료됨 — {err_log} 확인')

    # ---------- 콜백 ----------
    def _on_image(self, msg):
        self._latest = msg

    def _run_ocr(self, waypoint_id):
        """최신 프레임을 워커에 넘겨 (plate, conf, image_path, err) 받기."""
        msg = self._latest
        if msg is None:
            return '', 0.0, '', 'no camera frame'
        # rgb8 가정: (H,W,3) 로 복원
        arr = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        npy_path = os.path.join(self._capture_dir, 'frame.npy')
        np.save(npy_path, arr)
        png_path = os.path.join(self._capture_dir, f'{waypoint_id}.png')

        # 요청 전송 → RESULT 한 줄 수신
        req = json.dumps({'npy': npy_path, 'png': png_path})
        self._worker.stdin.write(req + '\n')
        self._worker.stdin.flush()
        for line in self._worker.stdout:
            if line.startswith('RESULT '):
                r = json.loads(line[len('RESULT '):])
                err = '' if r.get('ok') else r.get('error', 'no plate detected')
                return r.get('plate', ''), r.get('conf', 0.0), r.get('image_path', ''), err
        return '', 0.0, '', 'worker terminated'

    def execute_callback(self, goal_handle):
        waypoint_id = goal_handle.request.waypoint_id
        robot_pose = goal_handle.request.robot_pose
        self.get_logger().info(f'번호판 인식 요청: waypoint_id={waypoint_id}')

        fb = RecognizePlate.Feedback()
        fb.current_step = 'running_ocr'
        fb.progress = 0.3
        goal_handle.publish_feedback(fb)

        # 진짜 OCR
        plate, conf, image_path, err = self._run_ocr(waypoint_id)
        success = bool(plate)
        is_registered = success and (plate in self._registered)
        message = '' if success else err

        if success:
            tag = '등록' if is_registered else '미등록'
            self.get_logger().info(f'인식: {plate} (conf={conf:.3f}) → {tag}')
        else:
            self.get_logger().warn(f'인식 실패: {message}')

        # 토픽 발행 (monitor 로)
        det = PlateDetection()
        det.header.stamp = self.get_clock().now().to_msg()
        det.header.frame_id = 'map'
        det.waypoint_id = waypoint_id
        det.robot_pose = robot_pose
        det.plate_text = plate
        det.confidence = float(conf)
        det.is_registered = is_registered
        det.image_path = image_path
        det.success = success
        det.message = message
        self._plate_publisher.publish(det)

        goal_handle.succeed()
        result = RecognizePlate.Result()
        result.success = success
        result.message = message
        result.plate_text = plate
        result.confidence = float(conf)
        result.is_registered = is_registered
        result.image_path = image_path
        return result

    def destroy_node(self):
        if getattr(self, '_worker', None):
            try:
                self._worker.stdin.close()
                self._worker.terminate()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('perception_node 종료')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
