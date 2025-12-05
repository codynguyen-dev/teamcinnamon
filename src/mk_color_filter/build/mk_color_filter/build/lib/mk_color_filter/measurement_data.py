#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

import cv2
import numpy as np
from cv_bridge import CvBridge


class MeasurementData(Node):
    def __init__(self):
        super().__init__('measurement_data')

        # Parameters
        self.declare_parameter('mask_topic', '/image_filtered_mask')
        self.mask_topic = self.get_parameter('mask_topic').value

        # ROS interfaces
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, self.mask_topic, self.mask_callback, 10)
        self.pub = self.create_publisher(Float32MultiArray, '/sphere_stats', 10)

        self.get_logger().info(f"MeasurementData ready. Subscribing to: {self.mask_topic}")

    def mask_callback(self, msg: Image):
        try:
            mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        # Connected-components to detect blobs
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask)

        # Prepare output: [width, center_x]
        out = Float32MultiArray()

        if num_labels <= 1:
            # No blob detected — publish zeros
            out.data = [0.0, 0.0]
            self.pub.publish(out)
            return

        # Find largest blob (ignore background index 0)
        areas = stats[1:, cv2.CC_STAT_AREA]
        largest_index = 1 + np.argmax(areas)

        x = stats[largest_index, cv2.CC_STAT_LEFT]
        w = stats[largest_index, cv2.CC_STAT_WIDTH]
        cx = x + w / 2.0

        out.data = [float(w), float(cx)]  # [width_px, center_x_px]
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = MeasurementData()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
