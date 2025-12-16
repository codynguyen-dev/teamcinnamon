#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import termios
import tty

class KeyboardModeSwitch(Node):
    def __init__(self):
        super().__init__('keyboard_mode_switch')

        self.mode_pub = self.create_publisher(String, 'mode_request', 10)

        # save terminal settings
        self.settings = termios.tcgetattr(sys.stdin)

        self.get_logger().info('=== Keyboard Mode Switch ===')
        self.get_logger().info('Press:')
        self.get_logger().info('  E - Emergency Stop')
        self.get_logger().info('  T - Teleop Mode')
        self.get_logger().info('  A - Autonomous Mode')
        self.get_logger().info('  Q - Quit')
        self.get_logger().info('===========================')

        # timer to check for keypresses
        self.timer = self.create_timer(0.1, self.check_keypress)

    def get_key(self):
        """Get a single keypress without waiting for Enter"""
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def check_keypress(self):
        """Check for keypress and publish mode change"""
        try:
            key = self.get_key()
            key_lower = key.lower()

            if key_lower == 'e':
                msg = String()
                msg.data = 'e'
                self.mode_pub.publish(msg)
                self.get_logger().info('ESTOP requested')

            elif key_lower == 't':
                msg = String()
                msg.data = 't'
                self.mode_pub.publish(msg)
                self.get_logger().info('TELEOP requested')

            elif key_lower == 'a':
                msg = String()
                msg.data = 'a'
                self.mode_pub.publish(msg)
                self.get_logger().info('AUTONOMOUS requested')

            elif key_lower == 'q':
                self.get_logger().info('Quitting...')
                raise KeyboardInterrupt

        except Exception:
            pass

    def __del__(self):
        """Restore terminal settings on exit"""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardModeSwitch()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()