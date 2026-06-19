import os

# 로그 줄 포맷: "[INFO] [ts] [node]: msg" → 메시지만 (rclpy 로깅 초기화 전 설정)
os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{message}'

import rclpy
from rclpy.node import Node
from robochz_msgs.msg import PlateDetection


class MonitorNode(Node):                          # ① rclpy.node.Node 상속
    def __init__(self):
        super().__init__('monitor_node')          # ② ROS2에 보이는 노드 이름
        self.get_logger().info('monitor_node 시작')
        
        self.create_subscription(
            PlateDetection,
            '/plate_detection',
            self.plate_detection_callback,
            10
        )
    

    def plate_detection_callback(self, msg: PlateDetection):
        status = '등록' if msg.is_registered else '미등록'

        self.get_logger().info(
            f'[번호판 인식] '
            f'순찰지점={msg.waypoint_id}, '
            f'번호판={msg.plate_text}, '
            f'상태={status}'
        )


def main(args=None):
    rclpy.init(args=args)                         # ④ ROS2 통신 시스템 초기화
    node = MonitorNode()
    try:
        rclpy.spin(node)                          # ⑤ 콜백 받을 때까지 무한 대기
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('monitor_node 종료')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()