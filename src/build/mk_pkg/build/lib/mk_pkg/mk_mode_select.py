#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

HELP = """
Modes:
  a = AUTO
  t = TELEOP
  o = OBS (Obstacle Stop)
  e = ESTOP (latched; requires separate reset)
  h = help
  q = quit
"""

class ModeCLI(Node):
    def __init__(self):
        super().__init__('mk_mode_select')
        self.pub = self.create_publisher(String, '/mk/mode_request', 10)
        self.get_logger().info('Mode selector ready.\n' + HELP)

    def loop(self):
        while rclpy.ok():
            try:
                c = input('mode [a/t/o/e,h,q]> ').strip().lower()
            except (EOFError, KeyboardInterrupt):
                break

            mapping = {'a':'AUTO', 't':'TELEOP', 'o':'OBS', 'e':'ESTOP'}
            if c in mapping:
                msg = String(); msg.data = mapping[c]
                self.pub.publish(msg)
                self.get_logger().info(f'requested {mapping[c]}')
            elif c == 'h' or c == '?':
                print(HELP)
            elif c == 'q':
                break

def main():
    rclpy.init()
    node = ModeCLI()
    try:
        node.loop()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
