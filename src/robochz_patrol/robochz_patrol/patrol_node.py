import os
import math

# 로그 줄 포맷: "[INFO] [ts] [node]: msg" → 메시지만 (rclpy 로깅 초기화 전 설정)
os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{message}'

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from robochz_msgs.action import RecognizePlate

from nav2_msgs.action import NavigateThroughPoses
from action_msgs.msg import GoalStatus


class PatrolNode(Node):
    """Nav2 기반 순찰 — nav_through_poses 로 waypoint 이동 + 도착 시 인식 요청.
    (Rule-based 버전은 patrol_rulebased.py)
    """
    def __init__(self):
        super().__init__('patrol_node')
        self.get_logger().info('patrol_node 시작 (Nav2)')

        self._nav_client = ActionClient(self, NavigateThroughPoses,
                                        'navigate_through_poses')
        self._action_client = ActionClient(self, RecognizePlate, 'recognize_plate')

        # ── 순찰 waypoint 파라미터 ──
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
        self._wp_index = 0
        self.declare_parameter('settle_sec', 2.0)
        self._settle_sec = self.get_parameter('settle_sec').value
        self.get_logger().info(
            f'순찰 waypoint {len(self._waypoints)}개 로드 (도착 후 {self._settle_sec:.1f}s 대기)')

        # Timer 1회 트리거 → WP1 전송
        self._goal_sent = False
        self._startup_timer = self.create_timer(1.0, self._send_initial_goal_once)

    def _send_initial_goal_once(self):
        if self._goal_sent:
            return
        self._goal_sent = True
        self._startup_timer.cancel()
        self.send_navigation_goal()

    def send_navigation_goal(self):
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('navigate_through_poses 액션 서버 응답 없음')
            return
        x, y, yaw = self._waypoints[self._wp_index]
        target = PoseStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = 'map'
        target.pose.position.x = x
        target.pose.position.y = y
        target.pose.orientation.z = math.sin(yaw / 2.0)   # z축 회전만
        target.pose.orientation.w = math.cos(yaw / 2.0)
        goal = NavigateThroughPoses.Goal()
        goal.poses.append(target)
        self.get_logger().info(
            f'WP {self._wp_index + 1}/{len(self._waypoints)} '
            f'{self._wp_ids[self._wp_index]} ({x:.2f}, {y:.2f}) 이동 요청')
        future = self._nav_client.send_goal_async(
            goal, feedback_callback=self.nav_feedback_callback)
        future.add_done_callback(self.nav_goal_response_callback)

    def nav_goal_response_callback(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().info('Nav2 이동 요청 거부됨')
            return
        gh.get_result_async().add_done_callback(self.nav_get_result_callback)

    def nav_feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        # number_of_poses_remaining 은 goal당 pose 수(늘 1) → 우리 인덱스로 잔여 계산
        remaining = len(self._waypoints) - (self._wp_index + 1)
        self.get_logger().info(
            f'[{self._wp_ids[self._wp_index]}] 이동 중 · '
            f'남은거리 {fb.distance_remaining:.1f}m · 남은 {remaining}곳',
            throttle_duration_sec=3.0)

    def nav_get_result_callback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            wp = self._wp_ids[self._wp_index]
            self.get_logger().info(f'[{wp}] 도착 — {self._settle_sec:.1f}s 안정화 대기 후 인식')
            self._settle_timer = self.create_timer(self._settle_sec, self._after_settle)
        else:
            self.get_logger().error(f'Nav2 이동 실패, status={status}')

    def _after_settle(self):
        self._settle_timer.cancel()        # 일회성
        self.send_recognition_goal()

    def send_recognition_goal(self):
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('recognize_plate 액션 서버 응답 없음')
            return
        goal = RecognizePlate.Goal()
        goal.waypoint_id = self._wp_ids[self._wp_index]
        goal.robot_pose = PoseStamped()
        goal.robot_pose.header.stamp = self.get_clock().now().to_msg()
        goal.robot_pose.header.frame_id = 'map'
        goal.robot_pose.pose.orientation.w = 1.0
        future = self._action_client.send_goal_async(
            goal, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().info('번호판 인식 요청 거부됨')
            return
        gh.get_result_async().add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'인식 진행: {fb.current_step}, {fb.progress:.0%}')

    def get_result_callback(self, future):
        r = future.result().result
        tag = '등록' if r.is_registered else '미등록'
        self.get_logger().info(
            f'인식 결과: 번호판={r.plate_text}, 상태={tag}, 신뢰도={r.confidence:.2f}')
        # 다음 waypoint (비동기 콜백 체인)
        self._wp_index += 1
        if self._wp_index < len(self._waypoints):
            self.send_navigation_goal()
        else:
            self.get_logger().info('순찰 완료 — A구역 한 바퀴 종료')


def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('patrol_node 종료')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
