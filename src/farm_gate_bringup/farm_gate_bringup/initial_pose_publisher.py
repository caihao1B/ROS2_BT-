#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

def quat_from_yaw(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('topic', '/initialpose')
        self.declare_parameter('publish_count', 5)
        self.declare_parameter('publish_rate_hz', 1.0)

        self._x = float(self.get_parameter('x').value)
        self._y = float(self.get_parameter('y').value)
        self._yaw = float(self.get_parameter('yaw').value)
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._topic = str(self.get_parameter('topic').value)
        self._remaining = int(self.get_parameter('publish_count').value)
        rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self._done = False
        self._pub = self.create_publisher(PoseWithCovarianceStamped, self._topic, 10)
        period = 1.0 / max(rate_hz, 0.1)
        self._timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f'Publish /initialpose x={self._x:.3f}, y={self._y:.3f}, yaw={self._yaw:.3f} '
            f'count={self._remaining}, rate={rate_hz:.2f}Hz'
        )

    def _on_timer(self):
        if self._remaining <= 0:
            self._timer.cancel()
            self._done = True
            self.get_logger().info('Initial pose done. Exiting.')
            return

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.pose.pose.position.x = self._x
        msg.pose.pose.position.y = self._y
        msg.pose.pose.position.z = 0.0

        qx, qy, qz, qw = quat_from_yaw(self._yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        cov = [0.0] * 36
        cov[0] = 0.25
        cov[7] = 0.25
        cov[35] = (15.0 * math.pi / 180.0) ** 2
        msg.pose.covariance = cov

        self._pub.publish(msg)
        self._remaining -= 1
        self.get_logger().info(f'Published initial pose ({self._remaining} remaining)')

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    try:
        while rclpy.ok() and not node._done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
