import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from robochz_msgs.action import RecognizePlate
from robochz_msgs.msg import PlateDetection


class PerceptionNode(Node):                          # ① rclpy.node.Node 상속
    def __init__(self):
        super().__init__('perception_node')          # ② ROS2에 보이는 노드 이름
        self.get_logger().info('perception_node 시작')
        
        self._action_server = ActionServer(
            self,
            RecognizePlate,
            'recognize_plate',
            self.execute_callback
        )

        self._plate_publisher = self.create_publisher(
            PlateDetection,
            '/plate_detection',
            10
        )
    

    def execute_callback(self, goal_handle):
        # 1. Goal에서 요청 정보 꺼내기
        waypoint_id = goal_handle.request.waypoint_id
        robot_pose = goal_handle.request.robot_pose
        self.get_logger().info(f'번호판 인식 요청 수신: waypoint_id={waypoint_id}')
    
        # 2. Feedback 발행
        feedback = RecognizePlate.Feedback()
        feedback.current_step = 'running_ocr'
        feedback.progress = 0.3
        goal_handle.publish_feedback(feedback)
        
        # 3. 실제 번호판 인식 작업 수행 (여기서는 가짜로 sleep)
        self.get_logger().info('번호판 인식 시작')
        time.sleep(2)

        # 4. 가짜 OCR 결과
        plate_text = '12가3456'
        confidence = 0.9
        is_registered = False
        image_path = '/tmp/fake_plate.jpg'
        self.get_logger().info(f'번호판 인식 완료: {plate_text}')
        

        # 5. PlateDetection 토픽 발행
        detection = PlateDetection()
        detection.header.stamp = self.get_clock().now().to_msg()
        detection.header.frame_id = 'map'
        detection.waypoint_id = waypoint_id
        detection.robot_pose = robot_pose
        detection.plate_text = plate_text
        detection.confidence = confidence
        detection.is_registered = is_registered
        detection.image_path = image_path
        detection.success = True
        detection.message = ''

        self._plate_publisher.publish(detection)
        
        # 6. Result 반환
        goal_handle.succeed()
        result = RecognizePlate.Result()
        result.plate_text = plate_text
        result.is_registered = is_registered
        return result
    



def main(args=None):
    rclpy.init(args=args)                         # ④ ROS2 통신 시스템 초기화
    node = PerceptionNode()
    try:
        rclpy.spin(node)                          # ⑤ 콜백 받을 때까지 무한 대기
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('perception_node 종료')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()