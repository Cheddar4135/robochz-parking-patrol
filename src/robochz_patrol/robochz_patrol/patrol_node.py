import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from robochz_msgs.action import RecognizePlate

from nav2_msgs.action import NavigateThroughPoses
from action_msgs.msg import GoalStatus



class PatrolNode(Node):                          # ① rclpy.node.Node 상속
    def __init__(self):
        super().__init__('patrol_node')          # ② ROS2에 보이는 노드 이름
        self.get_logger().info('patrol_node 시작')

        # ── navigate_through_poses 액션 클라이언트 생성 ──
        self._nav_client = ActionClient(
            self,
            NavigateThroughPoses,
            'navigate_through_poses'
        )

        # ── recognize_plate 액션 클라이언트 생성 ──
        self._action_client = ActionClient(
            self,
            RecognizePlate,
            'recognize_plate'
        )

        # ── Timer 1회 트리거 패턴 ──
        # Iteration 0 테스트용:
        # 액션 클라이언트가 생성된 뒤 Goal을 1회만 보낸다. 
        self._goal_sent = False
        self._startup_timer = self.create_timer(
            1.0,
            self._send_initial_goal_once
        )

    def _send_initial_goal_once(self):
        # #timer는 기본적으로 반복 실행되므로, 테스트 Goal은 한 번만 보내고 중지한다.
        if self._goal_sent:
            return
        self._goal_sent = True
        self._startup_timer.cancel()

        # Iteration 0 
        # 먼저 waypoint로 이동한 뒤, 도착하면 번호판 인식을 요청한다.
        self.send_navigation_goal()

    def send_navigation_goal(self):
        self.get_logger().info('navigate_through_poses 액션 서버 대기...')

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                'navigate_through_poses 액션 서버 응답 없음'
            )
            return
        
        self.get_logger().info('navigate_through_poses 액션 서버 연결 완료')

        target = PoseStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = 'map'

        # Iteration 0 테스트용 waypoint 1개 
        target.pose.position.x = 1.0
        target.pose.position.y = 1.0
        target.pose.position.z = 0.0
        target.pose.orientation.w = 1.0

        goal = NavigateThroughPoses.Goal()
        goal.poses.append(target)

        self.get_logger().info('Nav2 waypoint 이동 요청 전송')

        future = self._nav_client.send_goal_async(
            goal,
            feedback_callback=self.nav_feedback_callback
        )
        future.add_done_callback(self.nav_goal_response_callback)

    def nav_goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Nav2 waypoint 이동 요청 거부됨')
            return
        self.get_logger().info('Nav2 waypoint 이동 요청 수락됨, 이동 중...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_get_result_callback)

    def nav_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        x = feedback.current_pose.pose.position.x
        y = feedback.current_pose.pose.position.y

        self.get_logger().info(
            f'Nav2 이동 중 | 위치=({x:.2f}, {y:.2f}) | '
            f'남은거리={feedback.distance_remaining:.2f}m | '
            f'남은 waypoint={feedback.number_of_poses_remaining}',
            throttle_duration_sec=1.0
        )

    def nav_get_result_callback(self, future):
        # Nav2 이동 결과 콜백에서는 액션의 최종 상태를 확인하여 성공 여부를 판단한다.
        status = future.result().status         
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Nav2 waypoint 이동 완료, 번호판 인식 요청 전송')
            self.send_recognition_goal()
        else:
            self.get_logger().error(
                f'Nav2 waypoint 이동 실패, status={status}'
            )

    def send_recognition_goal(self):
        # ── recognize_plate 액션 서버 확인 ──
        self.get_logger().info('recognize_plate 액션 서버 대기...')

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                'recognize_plate 액션 서버 응답 없음'
            )
            return

        self.get_logger().info('recognize_plate 액션 서버 연결 완료')

        # ── Goal 생성 ──
        # Iteration 0에서는 TEST_01 위치의 더미 번호판 인식 요청을 보낸다.
        goal = RecognizePlate.Goal()
        goal.waypoint_id = 'TEST_01'

        # robot_pose는 실제 로봇 위치를 담아야 하지만, 여기서는 더미 PoseStamped를 생성한다.
        goal.robot_pose = PoseStamped()
        goal.robot_pose.header.stamp = self.get_clock().now().to_msg()
        goal.robot_pose.header.frame_id = 'map'
        goal.robot_pose.pose.orientation.w = 1.0

        self.get_logger().info('번호판 인식 요청 전송')

        future = self._action_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )

        future.add_done_callback(
            self.goal_response_callback
        )

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('번호판 인식 요청 거부됨')
            return
        self.get_logger().info('번호판 인식 요청 수락됨, 결과 대기 중...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        # feedback_callback은 add_done_callback이 아니라
        # send_goal_async의 feedback_callback 인자로 직접 등록된다.
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'인식 진행 상황: {feedback.current_step}, 진행률={feedback.progress:.0%}'
        )

    def get_result_callback(self, future):
        result = future.result().result
        status = '등록' if result.is_registered else '미등록'

        self.get_logger().info(
            f'번호판 인식 결과: '
            f'번호판={result.plate_text}, '
            f'상태={status}, '
            f'신뢰도={result.confidence:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)                         # ④ ROS2 통신 시스템 초기화
    node = PatrolNode()
    try:
        rclpy.spin(node)                          # ⑤ 콜백 받을 때까지 무한 대기
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('patrol_node 종료')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
