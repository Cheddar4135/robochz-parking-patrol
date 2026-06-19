import os
import math

# 로그 줄 포맷: "[INFO] [ts] [node]: msg" → 메시지만 (rclpy 로깅 초기화 전 설정)
os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{message}'

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped, Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from robochz_msgs.action import RecognizePlate
from tf2_ros import StaticTransformBroadcaster


def patrol_status_qos():
    """순찰 종료 = 터미널 1회 신호 → transient_local(latched) 로 발행."""
    qos = QoSProfile(depth=1)
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    qos.reliability = ReliabilityPolicy.RELIABLE
    return qos


def norm(a):
    """각도를 [-π, π] 로 정규화."""
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(q):
    """쿼터니언 → yaw."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PatrolNode(Node):
    """Rule-based 순찰 — Nav2 대신 /odom 기반으로 직접 /cmd_vel 제어.
    시뮬 odom 이 정확하므로 결정론적 주행(드리프트/abort 없음).

    상태머신(각 waypoint): GOTO(위치로) → FACE(스캔 yaw로) → SETTLE(정지대기)
                          → WAIT_OCR(인식) → 다음.
    """
    def __init__(self):
        super().__init__('patrol_node')
        self.get_logger().info('patrol_node 시작 (rule-based)')

        # ── waypoint 파라미터 (map 프레임) ──
        self.declare_parameter('waypoints_x', [0.0])
        self.declare_parameter('waypoints_y', [0.0])
        self.declare_parameter('waypoints_yaw', [0.0])
        self.declare_parameter('waypoints_id', [''])
        xs = self.get_parameter('waypoints_x').value
        ys = self.get_parameter('waypoints_y').value
        yaws = self.get_parameter('waypoints_yaw').value
        self._waypoints = list(zip(xs, ys, yaws))
        ids = self.get_parameter('waypoints_id').value
        self._wp_ids = (list(ids) if len(ids) == len(self._waypoints)
                        else [f'A{i + 1}' for i in range(len(self._waypoints))])

        # ── 제어/스폰 파라미터 ──
        # odom 원점 = 스폰 위치(map 좌표). map 포즈 = 스폰 + odom (스폰 yaw 0 가정).
        self.declare_parameter('spawn_x', 11.85)
        self.declare_parameter('spawn_y', 8.0)
        self.declare_parameter('settle_sec', 5.0)
        self.declare_parameter('max_lin', 0.26)     # m/s (TB3 waffle 최대치, +~18%)
        self.declare_parameter('max_ang', 1.2)      # rad/s (+20%)
        self.declare_parameter('xy_tol', 0.12)      # 도착 반경(m)
        self.declare_parameter('yaw_tol', 0.05)     # 스캔 yaw 허용(rad)
        self._sx = self.get_parameter('spawn_x').value
        self._sy = self.get_parameter('spawn_y').value
        self._settle = self.get_parameter('settle_sec').value
        self._max_lin = self.get_parameter('max_lin').value
        self._max_ang = self.get_parameter('max_ang').value
        self._xy_tol = self.get_parameter('xy_tol').value
        self._yaw_tol = self.get_parameter('yaw_tol').value

        # ── I/O ──
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self._rec_client = ActionClient(self, RecognizePlate, 'recognize_plate')
        self._tf_bcast = StaticTransformBroadcaster(self)   # 보정된 map→odom 발행(RViz)
        self._status_pub = self.create_publisher(           # 완주 알림 → monitor 리포트
            Bool, '/patrol_status', patrol_status_qos())

        # ── 상태 ──
        self._pose = None          # (x, y, yaw) in map
        self._odom0 = None         # 스폰 시점 odom 기준점 (이중가산 방지·컨벤션 자동정합)
        self._wp_index = 0
        self._state = 'GOTO'
        self._settle_start = None
        self.get_logger().info(
            f'순찰 waypoint {len(self._waypoints)}개 (도착 후 {self._settle:.1f}s 대기)')

        # 20Hz 제어 루프
        self._timer = self.create_timer(0.05, self._control)

    # ---------- 콜백 ----------
    def _on_odom(self, msg):
        p = msg.pose.pose
        oyaw = yaw_of(p.orientation)
        if self._odom0 is None:
            # 시작 시점(정지 상태) odom 을 기준점으로 → 위치·방향 모두 자동정합.
            # (전제: 이때 로봇이 spawn(11.85,8, yaw0)에 있어야 함 → 재실행 전 순간이동)
            self._odom0 = (p.position.x, p.position.y, oyaw)
            self._publish_map_odom()
            kind = 'spawn 포함' if abs(p.position.x) > 1.0 else '0부터(spawn상대)'
            self.get_logger().info(
                f'odom0=({p.position.x:.2f},{p.position.y:.2f},yaw{oyaw:.2f}) '
                f'→ /odom 은 [{kind}]')
        ox0, oy0, oyaw0 = self._odom0
        # 기준점 대비 변위를 spawn 정렬 프레임으로 회전(odom 프레임이 oyaw0 만큼 돌아있을 수 있음)
        ddx, ddy = p.position.x - ox0, p.position.y - oy0
        c, s = math.cos(-oyaw0), math.sin(-oyaw0)
        self._pose = (self._sx + ddx * c - ddy * s,
                      self._sy + ddx * s + ddy * c,
                      norm(oyaw - oyaw0))   # spawn yaw 0 기준
    def _publish_map_odom(self):
        """시뮬 보정 map→odom = spawn ∘ odom0⁻¹ (RViz 표시용). 이중가산·방향 자동 해소."""
        ox0, oy0, oyaw0 = self._odom0
        c, s = math.cos(-oyaw0), math.sin(-oyaw0)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = self._sx - (ox0 * c - oy0 * s)
        t.transform.translation.y = self._sy - (ox0 * s + oy0 * c)
        t.transform.rotation.z = math.sin(-oyaw0 / 2.0)
        t.transform.rotation.w = math.cos(-oyaw0 / 2.0)
        self._tf_bcast.sendTransform(t)
        self.get_logger().info(
            f'map→odom 보정 발행 (odom0 yaw={oyaw0:.2f})')

    def _stop(self):
        self._cmd_pub.publish(Twist())

    def _drive(self, lin, ang):
        t = Twist()
        t.linear.x = max(-self._max_lin, min(self._max_lin, lin))
        t.angular.z = max(-self._max_ang, min(self._max_ang, ang))
        self._cmd_pub.publish(t)

    # ---------- 제어 루프 ----------
    def _control(self):
        if self._pose is None or self._wp_index >= len(self._waypoints):
            return
        rx, ry, ryaw = self._pose
        wx, wy, wyaw = self._waypoints[self._wp_index]

        if self._state == 'GOTO':
            dx, dy = wx - rx, wy - ry
            dist = math.hypot(dx, dy)
            if dist <= self._xy_tol:
                self._stop()
                self._state = 'FACE'
                return
            head_err = norm(math.atan2(dy, dx) - ryaw)
            if abs(head_err) > 0.25:           # 먼저 목표 방향으로 회전
                self._drive(0.0, 1.5 * head_err)
            else:                               # 정렬되면 전진(+미세 조향)
                self._drive(min(self._max_lin, 0.6 * dist), 1.0 * head_err)

        elif self._state == 'FACE':
            yaw_err = norm(wyaw - ryaw)
            if abs(yaw_err) <= self._yaw_tol:
                self._stop()
                self._settle_start = self.get_clock().now()
                self._state = 'SETTLE'
            else:
                self._drive(0.0, 1.2 * yaw_err)

        elif self._state == 'SETTLE':
            self._stop()
            elapsed = (self.get_clock().now() - self._settle_start).nanoseconds / 1e9
            if elapsed >= self._settle:
                self._state = 'WAIT_OCR'
                self._send_recognition()

        elif self._state == 'WAIT_OCR':
            self._stop()                        # 인식 동안 정지 유지

    # ---------- 인식 액션 ----------
    def _send_recognition(self):
        wp = self._wp_ids[self._wp_index]
        rx, ry, _ = self._pose
        self.get_logger().info(f'[{wp}] 도착·정지 → 번호판 인식 요청')
        if not self._rec_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('recognize_plate 서버 없음 → 다음 지점으로')
            self._advance()
            return
        goal = RecognizePlate.Goal()
        goal.waypoint_id = wp
        goal.robot_pose = PoseStamped()
        goal.robot_pose.header.frame_id = 'map'
        goal.robot_pose.header.stamp = self.get_clock().now().to_msg()
        goal.robot_pose.pose.position.x = rx
        goal.robot_pose.pose.position.y = ry
        goal.robot_pose.pose.orientation.w = 1.0
        self._rec_client.send_goal_async(goal).add_done_callback(self._goal_resp)

    def _goal_resp(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().warn('인식 요청 거부 → 다음 지점')
            self._advance()
            return
        gh.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        r = future.result().result
        tag = '등록' if r.is_registered else '미등록'
        self.get_logger().info(
            f'인식 결과: {r.plate_text or "(없음)"} [{tag}] conf={r.confidence:.2f}')
        self._advance()

    def _advance(self):
        self._wp_index += 1
        if self._wp_index < len(self._waypoints):
            self._state = 'GOTO'
        else:
            self._stop()
            self._state = 'DONE'
            self.get_logger().info('순찰 완료 — A구역 한 바퀴 종료')
            self._status_pub.publish(Bool(data=True))   # monitor 리포트 트리거


def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.get_logger().info('patrol_node 종료')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
