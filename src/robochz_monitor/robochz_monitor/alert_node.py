import os
import re
import csv
import subprocess
from datetime import datetime

# 한국 번호판 패턴: 숫자 2~3 + 한글 1 + 숫자 4 (예: 49다3433, 241허1861)
PLATE_RE = re.compile(r'^\d{2,3}[가-힣]\d{4}$')

# 로그 줄 포맷: "[INFO] [ts] [node]: msg" → 메시지만 (rclpy 로깅 초기화 전 설정)
os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{message}'

import rclpy
from rclpy.node import Node
from robochz_msgs.msg import PlateDetection

from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout, QFrame)
from PyQt5.QtGui import QPixmap, QFont
from PyQt5.QtCore import QTimer, Qt


class ViolationDialog(QWidget):
    """미등록 차량 1건 팝업. 확인=닫기 / 로그저장=CSV append."""

    def __init__(self, plate, waypoint, confidence, image_path, log_path, logger):
        super().__init__()
        self._plate = plate
        self._waypoint = waypoint
        self._confidence = confidence
        self._image_path = image_path
        self._log_path = log_path
        self._logger = logger
        self._saved = False

        self.setWindowTitle('CheezlBot Monitor')
        self.setMinimumWidth(320)
        root = QVBoxLayout(self)

        # 상태 헤더 (강조)
        status = QLabel('⚠ 상태: 미등록 차량 발견')
        status.setStyleSheet('color: #c0392b; font-weight: bold;')
        status.setFont(QFont('', 13))
        root.addWidget(status)
        root.addWidget(self._hline())

        # 정보 필드
        root.addWidget(QLabel(f'차량번호:  {plate}'))
        root.addWidget(QLabel(f'위치:      {waypoint}'))
        root.addWidget(QLabel(f'신뢰도:    {confidence * 100:.1f}%'))

        # 차량 사진
        img = QLabel()
        img.setAlignment(Qt.AlignCenter)
        img.setFrameShape(QFrame.Box)
        img.setMinimumHeight(180)
        pix = QPixmap(image_path) if image_path else QPixmap()
        if not pix.isNull():
            img.setPixmap(pix.scaledToWidth(360, Qt.SmoothTransformation))
        else:
            img.setText('(차량 사진 없음)')
        root.addWidget(img)

        # 버튼
        btns = QHBoxLayout()
        ok = QPushButton('확인')
        ok.clicked.connect(self.close)
        save = QPushButton('로그저장')
        save.clicked.connect(self._save_log)
        btns.addWidget(ok)
        btns.addWidget(save)
        root.addLayout(btns)

    def _hline(self):
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        return line

    def _save_log(self):
        """미등록 기록을 CSV 에 append (헤더 없으면 1회 생성)."""
        if self._saved:
            return
        os.makedirs(os.path.dirname(self._log_path), exist_ok=True)
        new = not os.path.exists(self._log_path)
        with open(self._log_path, 'a', newline='') as f:
            w = csv.writer(f)
            if new:
                w.writerow(['time', 'waypoint', 'plate', 'confidence', 'image_path'])
            w.writerow([datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                        self._waypoint, self._plate,
                        f'{self._confidence:.4f}', self._image_path])
        self._saved = True
        self._logger.info(f'미등록 로그 저장: {self._log_path} ({self._plate})')


class AlertNode(Node):
    """미등록 차량 즉시 알림 — /plate_detection 구독 → 소리(espeak) + PyQt 팝업.

    알림 규칙: OCR 텍스트가 한국 번호판 패턴(숫자2~3+한글1+숫자4)에 '정확히' 맞고
    + 등록 DB 에 없을 때만. 빈 칸/잡텍스트(패턴 불일치)는 무시.
    (worker 의 ok=bool(plate) 은 잡텍스트도 True 라 신뢰 불가 → 여기서 패턴 재검증)
    """

    def __init__(self):
        super().__init__('alert_node')
        self.get_logger().info('alert_node 시작 (미등록 즉시 알림)')

        home = os.path.expanduser('~/workspace/ros2_ws/robochz_ws')
        self.declare_parameter('violations_log', os.path.join(home, 'log', 'violations.csv'))
        # 음성: espeak-ng 영어. 미설치 시 graceful. 파라미터로 교체 가능(한국어: voice=ko 등).
        self.declare_parameter('speak_cmd', 'espeak-ng')
        self.declare_parameter('speak_voice', 'en')
        self.declare_parameter('speak_text', 'Unregistered vehicle detected.')
        self.declare_parameter('speak_rate', 140)   # 말속도(wpm). 기본 175보다 느려 또렷
        self._violations_log = self.get_parameter('violations_log').value
        self._speak_cmd = self.get_parameter('speak_cmd').value
        self._speak_voice = self.get_parameter('speak_voice').value
        self._speak_text = self.get_parameter('speak_text').value
        self._speak_rate = self.get_parameter('speak_rate').value

        self._dialogs = []   # 팝업 GC 방지로 참조 보관

        self.create_subscription(
            PlateDetection, '/plate_detection', self._on_detection, 10)

    def _on_detection(self, msg: PlateDetection):
        plate = (msg.plate_text or '').strip()
        # 번호판 패턴 불일치(빈 칸·잡텍스트) 무시 + 등록 차량 무시 → 미등록만 알림
        if not PLATE_RE.fullmatch(plate):
            return
        if msg.is_registered:
            return
        self.get_logger().warn(
            f'★ 미등록 차량 발견: {msg.plate_text} @ {msg.waypoint_id} '
            f'(conf={msg.confidence:.2f})')
        self._speak()
        self._popup(msg)

    def _speak(self):
        """리눅스 espeak 로 음성 알림. 미설치/실패 시 경고만 하고 진행."""
        try:
            subprocess.Popen(
                [self._speak_cmd, '-v', self._speak_voice,
                 '-s', str(self._speak_rate), self._speak_text],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except FileNotFoundError:
            self.get_logger().warn(
                f"'{self._speak_cmd}' 미설치 → 음성 생략 (sudo apt install espeak)")
        except Exception as e:
            self.get_logger().warn(f'음성 알림 실패: {e}')

    def _popup(self, msg: PlateDetection):
        dlg = ViolationDialog(
            plate=msg.plate_text, waypoint=msg.waypoint_id,
            confidence=msg.confidence, image_path=msg.image_path,
            log_path=self._violations_log, logger=self.get_logger())
        dlg.show()
        dlg.raise_()
        dlg.activateWindow()
        self._dialogs.append(dlg)


def main(args=None):
    rclpy.init(args=args)
    app = QApplication.instance() or QApplication([])  # Qt 메인 이벤트루프
    # 팝업을 닫아도 앱이 종료되지 않게 → 이후 미등록 차량도 계속 알림
    app.setQuitOnLastWindowClosed(False)
    node = AlertNode()

    # QTimer 로 ROS 콜백을 GUI 스레드에서 주기 처리 → 위젯 생성이 스레드-안전
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0))
    timer.start(50)   # 20Hz

    try:
        app.exec_()
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('alert_node 종료')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
