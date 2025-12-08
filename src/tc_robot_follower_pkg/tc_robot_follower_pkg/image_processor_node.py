#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageProcessorNode(Node):
    def __init__(self):
        super().__init__('image_processor_node')
        
        # Declare parameters for HSV tuning
        self.declare_parameter('hsv_low_h', 100)
        self.declare_parameter('hsv_low_s', 50)
        self.declare_parameter('hsv_low_v', 50)
        self.declare_parameter('hsv_high_h', 130)
        self.declare_parameter('hsv_high_s', 255)
        self.declare_parameter('hsv_high_v', 255)
        
        # Erosion/dilation parameters
        self.declare_parameter('erode_iterations', 2)
        self.declare_parameter('dilate_iterations', 2)
        
        # Minimum contour area (to filter noise)
        self.declare_parameter('min_contour_area', 100.0)
        
        self.bridge = CvBridge()
        
        # Subscribe to camera
        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',  # No leading slash! Remap this to actual camera topic
            self.image_callback,
            10
        )
        
        # Publish filtered/annotated image for debugging
        self.debug_image_pub = self.create_publisher(
            Image,
            'debug_image',
            10
        )
        
        # Publish object measurements for PID controller
        self.measurements_pub = self.create_publisher(
            Float32MultiArray,
            'object_measurements',
            10
        )
        
        self.get_logger().info('Image Processor Node started')
        self.get_logger().info('Subscribed to: camera/image_raw')
        self.get_logger().info('Publishing to: debug_image, object_measurements')
    
    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Get current HSV parameters
            lower_hsv = np.array([
                self.get_parameter('hsv_low_h').value,
                self.get_parameter('hsv_low_s').value,
                self.get_parameter('hsv_low_v').value
            ])
            upper_hsv = np.array([
                self.get_parameter('hsv_high_h').value,
                self.get_parameter('hsv_high_s').value,
                self.get_parameter('hsv_high_v').value
            ])
            
            # Convert to HSV and create mask
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv_image, lower_hsv, upper_hsv)
            
            # Morphological operations to reduce noise
            kernel = np.ones((5, 5), np.uint8)
            erode_iter = self.get_parameter('erode_iterations').value
            dilate_iter = self.get_parameter('dilate_iterations').value
            
            mask = cv2.erode(mask, kernel, iterations=erode_iter)
            mask = cv2.dilate(mask, kernel, iterations=dilate_iter)
            
            # Find contours
            contours, _ = cv2.findContours(
                mask,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE
            )
            
            # Get image dimensions
            height, width = cv_image.shape[:2]
            image_center_x = width / 2.0
            
            # Create debug image (color with mask overlay)
            debug_image = cv_image.copy()
            cv2.line(debug_image, (int(image_center_x), 0), 
                    (int(image_center_x), height), (0, 255, 0), 2)
            
            # Filter contours by minimum area
            min_area = self.get_parameter('min_contour_area').value
            valid_contours = [c for c in contours if cv2.contourArea(c) > min_area]
            
            if len(valid_contours) == 0:
                # No object detected - publish zeros
                measurements = Float32MultiArray()
                measurements.data = [0.0, 0.0, 0.0]  # [x_error, size, detected_flag]
                self.measurements_pub.publish(measurements)
                
                # Publish debug image
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
                debug_msg.header = msg.header
                self.debug_image_pub.publish(debug_msg)
                return
            
            # Find largest contour
            largest_contour = max(valid_contours, key=cv2.contourArea)
            
            # Calculate moments for centroid
            M = cv2.moments(largest_contour)
            
            if M['m00'] == 0:
                measurements = Float32MultiArray()
                measurements.data = [0.0, 0.0, 0.0]
                self.measurements_pub.publish(measurements)
                return
            
            # Centroid position
            cx = M['m10'] / M['m00']
            cy = M['m01'] / M['m00']
            
            # Calculate horizontal error (pixels from center)
            # Positive = object is to the right, negative = left
            horizontal_error = cx - image_center_x
            
            # Get contour area as size metric
            object_area = cv2.contourArea(largest_contour)
            
            # Draw on debug image
            cv2.drawContours(debug_image, [largest_contour], -1, (0, 255, 0), 3)
            cv2.circle(debug_image, (int(cx), int(cy)), 10, (0, 0, 255), -1)
            cv2.putText(debug_image, f'X_err: {horizontal_error:.1f}px', 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(debug_image, f'Area: {object_area:.1f}px^2', 
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Publish measurements: [horizontal_error, object_area, detection_flag]
            measurements = Float32MultiArray()
            measurements.data = [float(horizontal_error), float(object_area), 1.0]
            self.measurements_pub.publish(measurements)
            
            # Publish debug image
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
            debug_msg.header = msg.header
            self.debug_image_pub.publish(debug_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
