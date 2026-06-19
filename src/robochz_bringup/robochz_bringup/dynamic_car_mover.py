import os
import math

# 로그 줄 포맷: 메시지만 (rclpy 로깅 초기화 전)
os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{message}'

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def norm(a):
    """각도를 [-π, π] 로 정규화."""
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class DynamicCarMover(Node):
    """고정경로 동적 차량 — planar_move 차량을 cmd_vel 로 경로추종.

    텔레포트(kinematic) 대신 **진짜 물리 바디**라 정적 장애물과 정상 충돌(안 뚫음),
    로봇은 LiDAR/costmap 으로 회피(불도저 아님). Chaikin 으로 코너를 둥글게 +
    회전속도 제한으로 차처럼 완만하게 주행.
    """

    def __init__(self):
        super().__init__('dynamic_car_mover')

        self.declare_parameter('cmd_vel_topic', '/dyn_car/cmd_vel')
        self.declare_parameter('odom_topic', '/dyn_car/odom')
        self.declare_parameter('speed', 0.39)        # m/s (TB3 0.26 × 1.5)
        self.declare_parameter('max_wz', 0.7)        # rad/s 회전속도 상한(완만)
        self.declare_parameter('k_head', 1.5)        # 방향오차 P 이득
        self.declare_parameter('start_delay', 5.0)
        self.declare_parameter('advance_radius', 1.0)  # 다음 점으로 넘어가는 거리
        self.declare_parameter('goal_tol', 0.4)
        self.declare_parameter('smooth_iters', 4)
        self.declare_parameter('path_x', [0.0, 0.0, 24.0, 24.0])
        self.declare_parameter('path_y', [0.0, 9.5, 9.5, 20.0])

        self._speed = self.get_parameter('speed').value
        self._max_wz = self.get_parameter('max_wz').value
        self._k = self.get_parameter('k_head').value
        self._delay = self.get_parameter('start_delay').value
        self._adv = self.get_parameter('advance_radius').value
        self._goal_tol = self.get_parameter('goal_tol').value
        xs = self.get_parameter('path_x').value
        ys = self.get_parameter('path_y').value
        raw = list(zip(xs, ys))
        if len(raw) < 2:
            raise RuntimeError('path 는 최소 2점 필요')
        iters = self.get_parameter('smooth_iters').value
        self._pts = self._chaikin(raw, iters) if iters > 0 and len(raw) > 2 else raw

        self._pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self._on_odom, 10)

        self._pose = None
        self._t0 = None
        self._idx = 0
        self._done = False
        self._timer = self.create_timer(0.05, self._tick)   # 20Hz
        self.get_logger().info(
            f'동적 차량 경로추종 시작: smooth {len(self._pts)}점, '
            f'{self._speed:.2f}m/s, 출발지연 {self._delay:.0f}s')

    @staticmethod
    def _chaikin(pts, iters):
        """Chaikin 코너커팅 — 각 변을 1/4·3/4 지점으로 잘라 코너를 둥글게. 끝점 보존."""
        for _ in range(iters):
            new = [pts[0]]
            for i in range(len(pts) - 1):
                (px, py), (qx, qy) = pts[i], pts[i + 1]
                new.append((0.75 * px + 0.25 * qx, 0.75 * py + 0.25 * qy))
                new.append((0.25 * px + 0.75 * qx, 0.25 * py + 0.75 * qy))
            new.append(pts[-1])
            pts = new
        return pts

    def _on_odom(self, msg):
        p = msg.pose.pose
        self._pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def _stop(self):
        self._pub.publish(Twist())

    def _tick(self):
        if self._pose is None or self._done:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now == 0.0:
            return
        if self._t0 is None:
            self._t0 = now
        if now - self._t0 < self._delay:
            self._stop()              # 대기
            return

        x, y, yaw = self._pose
        # 다음 목표점으로 진행 (도달반경 내면 인덱스 전진)
        while (self._idx < len(self._pts) - 1
               and math.hypot(self._pts[self._idx][0] - x,
                              self._pts[self._idx][1] - y) < self._adv):
            self._idx += 1
        tx, ty = self._pts[self._idx]
        dist = math.hypot(tx - x, ty - y)

        # 종료 판정 (마지막 점 도달)
        if self._idx == len(self._pts) - 1 and dist < self._goal_tol:
            self._stop()
            self._done = True
            self.get_logger().info('동적 차량 경로 종료 — 정지')
            return

        head_err = norm(math.atan2(ty - y, tx - x) - yaw)
        cmd = Twist()
        cmd.angular.z = max(-self._max_wz, min(self._max_wz, self._k * head_err))
        # 방향오차 클수록 감속 → 코너에서 완만(차처럼)
        cmd.linear.x = self._speed * max(0.15, math.cos(head_err))
        self._pub.publish(cmd)
        self.get_logger().info(
            f'[동적차량] x={x:.1f} y={y:.1f} → 목표({tx:.1f},{ty:.1f})',
            throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicCarMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
