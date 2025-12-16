#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from enum import Enum

class RobotMode(Enum):
    EMERGENCY_STOP = 1
    TELEOPERATED = 2
    AUTONOMOUS = 3

class FSMNode(Node):
    def __init__(self):
        super().__init__('fsm_node')

        # start in ESTOP for safety
        self.current_mode = RobotMode.EMERGENCY_STOP

        # store last commands
        self.last_autonomous_cmd = Twist()
        self.last_teleop_cmd = Twist()

        # subscribers
        # autonomous commands from PID controller
        self.autonomous_sub = self.create_subscription(
            Twist,
            'autonomous_cmd',  # no leading slash!
            self.autonomous_callback,
            10
        )

        # teleop_twist_keyboard commands
        self.teleop_sub = self.create_subscription(
            Twist,
            'teleop_cmd',
            self.teleop_callback,
            10
        )

        # mode requests (single character: 'e', 't', 'a')
        self.mode_request_sub = self.create_subscription(
            String,
            'mode_request',  # no leading slash!!
            self.mode_request_callback,
            10
        )

        # publisher
        # output to motor controller
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/abot/cmd_vel',  # NO LEADING SLASH!!
            10
        )

        self.get_logger().info('FSM Node initialized in EMERGENCY_STOP mode')
        self.get_logger().info('Send "e" for ESTOP, "t" for Teleop, "a" for Autonomous')

    def autonomous_callback(self, msg):
        """Store autonomous command, forward if in AUTO mode"""
        self.last_autonomous_cmd = msg
        if self.current_mode == RobotMode.AUTONOMOUS:
            self.cmd_vel_pub.publish(msg)

    def teleop_callback(self, msg):
        """Store teleop command, forward if in TELEOP mode"""
        self.last_teleop_cmd = msg
        if self.current_mode == RobotMode.TELEOPERATED:
            self.cmd_vel_pub.publish(msg)

    def mode_request_callback(self, msg):
        """Handle mode switching via single keypress"""
        mode_char = msg.data.lower().strip()

        old_mode = self.current_mode

        if mode_char == 'e':
            self.current_mode = RobotMode.EMERGENCY_STOP
        elif mode_char == 't':
            self.current_mode = RobotMode.TELEOPERATED
        elif mode_char == 'a':
            self.current_mode = RobotMode.AUTONOMOUS
        else:
            self.get_logger().warn(f'Unknown mode: {mode_char}. Use e/t/a')
            return

        if old_mode != self.current_mode:
            self.handle_mode_transition(old_mode)

    def handle_mode_transition(self, old_mode):
        """Execute actions when changing modes"""

        # send stop command when entering ESTOP
        if self.current_mode == RobotMode.EMERGENCY_STOP:
            stop_cmd = Twist()  # All zeros
            for _ in range(5):  # send multiple times to ensure it's received
                self.cmd_vel_pub.publish(stop_cmd)
            self.get_logger().info('EMERGENCY STOP ACTIVE')

        elif self.current_mode == RobotMode.TELEOPERATED:
            # send stop, then wait for teleop commands
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
            self.get_logger().info('TELEOP MODE - Manual control enabled')

        elif self.current_mode == RobotMode.AUTONOMOUS:
            self.get_logger().info('AUTONOMOUS MODE - Robot following object')

        self.get_logger().info(f'Mode: {self.current_mode.name}')

def main(args=None):
    rclpy.init(args=args)
    node = FSMNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # stop before shutting down
        stop = Twist()
        node.cmd_vel_pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()