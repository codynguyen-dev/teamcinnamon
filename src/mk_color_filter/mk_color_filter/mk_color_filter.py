#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

from sensor_msgs.msg import Image
from std_msgs.msg import String

import cv2
import numpy as np
from cv_bridge import CvBridge


class ColorFilterNode(Node):
    def __init__(self):
        super().__init__('color_filter_node')

        self.declare_parameter('target_color', 'blue')
        self.declare_parameter('kernel_size', 5)           
        self.declare_parameter('iterations', 2)            
        self.declare_parameter('in_image_topic', '/image_raw')
        self.declare_parameter('out_image_topic', '/image_filtered')
        self.declare_parameter('publish_mask_debug', True) 

        self.color_ranges = {
            'blue':  [(100, 120,  60), (130, 255, 255)],  
            'pink':  [(145,  80, 120), (175, 255, 255)],  
            'green': [( 35,  60,  60), (85, 255, 255)],
        }

        self.target_color = self.get_parameter('target_color').get_parameter_value().string_value
        if self.target_color not in self.color_ranges:
            self.get_logger().warn(f"Unknown target_color '{self.target_color}', defaulting to 'blue'.")
            self.target_color = 'blue'

        self.kernel_size = int(self.get_parameter('kernel_size').value)
        if self.kernel_size % 2 == 0:
            self.kernel_size += 1
        self.iterations = max(0, int(self.get_parameter('iterations').value))
        self.in_topic = self.get_parameter('in_image_topic').value
        self.out_topic = self.get_parameter('out_image_topic').value
        self.publish_mask_debug = bool(self.get_parameter('publish_mask_debug').value)

        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, self.in_topic, self.image_cb, 10)
        self.pub = self.create_publisher(Image, self.out_topic, 10)
        self.mask_pub = self.create_publisher(Image, f"{self.out_topic}_mask", 10) if self.publish_mask_debug else None
        self.state_pub = self.create_publisher(String, f"{self.get_name()}/state", 10)

        self.add_on_set_parameters_callback(self.on_param_change)

        self.get_logger().info(
            f"ColorFilterNode ready. Subscribing: {self.in_topic} | Publishing: {self.out_topic} | target_color={self.target_color}"
        )

    def on_param_change(self, params):
        for p in params:
            if p.name == 'target_color' and p.type_ == Parameter.Type.STRING:
                new_color = p.value
                if new_color not in self.color_ranges:
                    self.get_logger().error(f"Rejected target_color '{new_color}'. Valid: {list(self.color_ranges)}")
                    return SetParametersResult(successful=False)
                self.target_color = new_color
                self.get_logger().info(f"Switched target_color to: {self.target_color}")
            elif p.name == 'kernel_size':
                ks = int(p.value)
                if ks < 1:
                    return SetParametersResult(successful=False)
                self.kernel_size = ks if ks % 2 == 1 else ks + 1
                self.get_logger().info(f"kernel_size set to: {self.kernel_size}")
            elif p.name == 'iterations':
                it = int(p.value)
                if it < 0:
                    return SetParametersResult(successful=False)
                self.iterations = it
                self.get_logger().info(f"iterations set to: {self.iterations}")
        return SetParametersResult(successful=True)

    def image_cb(self, msg: Image):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        lo, hi = self.color_ranges[self.target_color]
        lo = np.array(lo, dtype=np.uint8)
        hi = np.array(hi, dtype=np.uint8)

        mask = cv2.inRange(hsv, lo, hi)

        if self.iterations > 0 and self.kernel_size > 0:
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (self.kernel_size, self.kernel_size))
            mask = cv2.erode(mask, kernel, iterations=self.iterations)
            mask = cv2.dilate(mask, kernel, iterations=self.iterations)

        result = cv2.bitwise_and(bgr, bgr, mask=mask)

        out_msg = self.bridge.cv2_to_imgmsg(result, encoding='bgr8')
        out_msg.header = msg.header
        self.pub.publish(out_msg)

        if self.mask_pub is not None:
            mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
            mask_msg.header = msg.header
            self.mask_pub.publish(mask_msg)

        self.state_pub.publish(String(data=self.target_color))


def main(args=None):
    rclpy.init(args=args)
    node = ColorFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
