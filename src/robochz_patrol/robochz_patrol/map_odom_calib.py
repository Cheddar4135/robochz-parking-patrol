import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import StaticTransformBroadcaster


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class MapOdomCalib(Node):
    """첫 /odom 을 기준으로 정적 map→odom = spawn ∘ odom0⁻¹ 를 1회 발행.

    Gazebo diff_drive 의 /odom 이 spawn world 위치를 포함하든(world) 0부터든,
    시작 시점(로봇이 spawn 에 정지)의 odom 을 기준점으로 빼서 올바른 TF를 산출 →
    'spawn 을 두 번 더하는' 이중가산을 자동으로 막는다. (시뮬 완벽 localization)

    전제: 이 노드 시작 때 로봇이 spawn(spawn_x, spawn_y, yaw0)에 정지해 있어야 함.
    """
    def __init__(self):
        super().__init__('map_odom_calib')
        self.declare_parameter('spawn_x', 11.85)
        self.declare_parameter('spawn_y', 8.0)
        self._sx = self.get_parameter('spawn_x').value
        self._sy = self.get_parameter('spawn_y').value
        self._bcast = StaticTransformBroadcaster(self)
        self._done = False
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.get_logger().info('map_odom_calib: 첫 odom 대기...')

    def _on_odom(self, msg):
        if self._done:
            return
        self._done = True
        p = msg.pose.pose
        ox, oy, oyaw = p.position.x, p.position.y, yaw_of(p.orientation)
        # map→odom = spawn ∘ odom0⁻¹  (회전 -oyaw 포함)
        c, s = math.cos(-oyaw), math.sin(-oyaw)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = self._sx - (ox * c - oy * s)
        t.transform.translation.y = self._sy - (ox * s + oy * c)
        t.transform.rotation.z = math.sin(-oyaw / 2.0)
        t.transform.rotation.w = math.cos(-oyaw / 2.0)
        self._bcast.sendTransform(t)
        kind = 'spawn 포함(world)' if abs(ox) > 1.0 else '0부터(spawn상대)'
        self.get_logger().info(
            f'odom0=({ox:.2f},{oy:.2f},yaw{oyaw:.2f}) [{kind}] → '
            f'map→odom=({t.transform.translation.x:.2f},{t.transform.translation.y:.2f})')


def main(args=None):
    rclpy.init(args=args)
    node = MapOdomCalib()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
