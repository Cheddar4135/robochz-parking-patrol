import os
import csv
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf2_ros import Buffer, TransformListener


def _stamp_sec(stamp):
    """builtin_interfaces/Time → float 초."""
    return stamp.sec + stamp.nanosec * 1e-9


class PathRecorder(Node):
    """로봇 주행궤적 기록·시각화.
    TF(map→base_footprint)를 주기적으로 읽어 nav_msgs/Path(/patrol_path)로 누적 발행
    → RViz 'Path' 디스플레이로 전체 경로 시각화. 종료 시 CSV 저장(보고서용).
    """
    def __init__(self):
        super().__init__('path_recorder')
        default_csv = os.path.expanduser(
            '~/workspace/ros2_ws/robochz_ws/log/robochz_captures/patrol_path.csv')
        self.declare_parameter('out_csv', default_csv)
        self.declare_parameter('min_step', 0.03)   # 이 거리 이상 움직였을 때만 점 추가(m)
        self._out = self.get_parameter('out_csv').value
        self._min_step = self.get_parameter('min_step').value

        self._buf = Buffer()
        self._listener = TransformListener(self._buf, self)
        self._path = Path()
        self._path.header.frame_id = 'map'
        self._pub = self.create_publisher(Path, '/patrol_path', 10)
        self._timer = self.create_timer(0.2, self._tick)   # 5Hz
        self.get_logger().info('path_recorder: /patrol_path 기록 시작 (RViz Path 디스플레이로 확인)')

    def _tick(self):
        try:
            tf = self._buf.lookup_transform('map', 'base_footprint', Time())
        except Exception:
            return   # 아직 TF 없음
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        if self._path.poses:
            last = self._path.poses[-1].pose.position
            if math.hypot(x - last.x, y - last.y) < self._min_step:
                return   # 거의 안 움직임 → 스킵
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation = tf.transform.rotation
        self._path.poses.append(ps)
        self._path.header.stamp = ps.header.stamp
        self._pub.publish(self._path)

    def save(self):
        if not self._path.poses:
            self.get_logger().warn('저장할 경로 없음')
            return
        os.makedirs(os.path.dirname(self._out), exist_ok=True)
        t0 = _stamp_sec(self._path.poses[0].header.stamp)
        with open(self._out, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'x', 'y'])      # t = 첫 점 기준 상대시각(초)
            for p in self._path.poses:
                t = _stamp_sec(p.header.stamp) - t0
                w.writerow([f'{t:.3f}', f'{p.pose.position.x:.4f}',
                            f'{p.pose.position.y:.4f}'])
        self.get_logger().info(f'주행경로 {len(self._path.poses)}점 저장: {self._out}')


def main(args=None):
    rclpy.init(args=args)
    node = PathRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
