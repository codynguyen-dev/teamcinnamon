#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class EstopResetCLI(Node):
    def __init__(self):
        super().__init__('mk_estop_reset')
        self.pub = self.create_publisher(Bool, '/mk/estop_reset', 10)
        self.get_logger().info('Press Enter to RESET the ESTOP latch. Ctrl-C to exit.')

    def loop(self):
        while rclpy.ok():
            try:
                input('reset estop? [Enter] ')
            except (EOFError, KeyboardInterrupt):
                break
            self.pub.publish(Bool(data=True))
            self.get_logger().info('ESTOP reset command sent.')

def main():
    rclpy.init()
    node = EstopResetCLI()
    try:
        node.loop()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
