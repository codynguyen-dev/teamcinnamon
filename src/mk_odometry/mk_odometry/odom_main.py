#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from odometry_helper_msg.msg import DistWheel
from turtlesim.msg import Pose

def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))

class DiffDriveOdom(Node):
    def __init__(self):
        super().__init__('odometer')
        self.declare_parameter('L', 0.05)
        self.L = float(self.get_parameter('L').value)
        self.x = 0.0; self.y = 0.0; self.th = 0.0

        self.sub = self.create_subscription(DistWheel, 'dist_wheel', self.cb, 10)
        self.pub = self.create_publisher(Pose, 'pose', 10)

        self.get_logger().info('[mk_odom] sub=dist_wheel (DistWheel), pub=pose (turtlesim/Pose)')

    def cb(self, msg):
        dl = float(msg.dist_wheel_left)
        dr = float(msg.dist_wheel_right)

        v = 0.5 * (dl + dr)
        dth = (dr - dl) / (2.0 * self.L)
        th_mid = self.th + 0.5 * dth

        self.x += v * math.cos(th_mid)
        self.y += v * math.sin(th_mid)
        self.th = wrap(self.th + dth)

        p = Pose()
        p.x = self.x
        p.y = self.y
        p.theta = self.th
        p.linear_velocity = 0.0
        p.angular_velocity = 0.0
        self.pub.publish(p)

def main():
    rclpy.init()
    n = DiffDriveOdom()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()