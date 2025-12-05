#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from turtlesim.msg import Pose

class Distance(Node):
    def __init__(self):
        super().__init__('distance')
        self.pub = self.create_publisher(Float32, 'distance', 10)
        self.sub = self.create_subscription(Pose, 'turtle1/pose', self.pose_cb, 10)

        self._last_x = None
        self._last_y = None
        self._total_dist = 0.0
        self._out_msg = Float32()

        self.get_logger().info('node=distance status=ready pub=distance sub=turtle1/pose')

    def pose_cb(self, msg: Pose):
        if self._last_x is None:
            self._last_x, self._last_y = msg.x, msg.y
            self._publish()
            return

        dx = msg.x - self._last_x
        dy = msg.y - self._last_y
        self._total_dist += math.hypot(dx, dy)
        self._last_x, self._last_y = msg.x, msg.y
        self._publish()

    def _publish(self):
        # Only publish if the node is still alive
        if not rclpy.ok():
            return
        self._out_msg.data = float(self._total_dist)
        self.pub.publish(self._out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Distance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutdown requested by user (Ctrl+C)')
    finally:
        # Only shut down once, if still active
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
