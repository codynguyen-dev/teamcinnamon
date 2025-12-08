#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

class DualPIDController(Node):
    def __init__(self):
        super().__init__('dual_pid_controller')
        
        # ===== ANGULAR CONTROLLER (turns to face object) =====
        self.declare_parameter('angular_kp', 0.005)
        self.declare_parameter('angular_ki', 0.0)
        self.declare_parameter('angular_kd', 0.001)
        self.declare_parameter('angular_max_output', 1.0)
        
        # ===== LINEAR CONTROLLER (maintains distance) =====
        self.declare_parameter('linear_kp', 0.0001)
        self.declare_parameter('linear_ki', 0.0)
        self.declare_parameter('linear_kd', 0.0)
        self.declare_parameter('linear_max_output', 0.3)
        
        # Target object size (area in pixels^2) - tune this!
        self.declare_parameter('target_object_area', 8000.0)
        
        # Deadband - don't move if error is small
        self.declare_parameter('angular_deadband', 10.0)  # pixels
        self.declare_parameter('linear_deadband', 500.0)  # area pixels
        
        # Angular controller state
        self.angular_error_sum = 0.0
        self.angular_last_error = None
        self.angular_last_time = None
        
        # Linear controller state
        self.linear_error_sum = 0.0
        self.linear_last_error = None
        self.linear_last_time = None
        
        # Subscribe to object measurements
        self.measurements_sub = self.create_subscription(
            Float32MultiArray,
            'object_measurements',
            self.measurements_callback,
            10
        )
        
        # Publish autonomous commands
        self.cmd_pub = self.create_publisher(
            Twist,
            'autonomous_cmd',
            10
        )
        
        self.get_logger().info('Dual PID Controller started')
        self.get_logger().info('Waiting for object measurements...')
    
    def measurements_callback(self, msg):
        """
        Receives: [horizontal_error (px), object_area (px^2), detected_flag]
        Outputs: Twist message with angular.z and linear.x
        """
        
        if len(msg.data) < 3:
            self.get_logger().warn('Invalid measurement message')
            return
        
        horizontal_error = msg.data[0]  # Positive = right, negative = left
        object_area = msg.data[1]
        detected = msg.data[2]
        
        # If no object detected, send stop command
        if detected == 0.0:
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            
            # Reset integrators when no detection
            self.angular_error_sum = 0.0
            self.linear_error_sum = 0.0
            self.angular_last_error = None
            self.linear_last_error = None
            return
        
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # ===== ANGULAR CONTROL (turning) =====
        angular_output = self.compute_angular_control(horizontal_error, current_time)
        
        # ===== LINEAR CONTROL (forward/backward) =====
        linear_output = self.compute_linear_control(object_area, current_time)
        
        # Create and publish Twist message
        cmd = Twist()
        cmd.angular.z = angular_output  # Turning
        cmd.linear.x = linear_output    # Forward/backward
        
        self.cmd_pub.publish(cmd)
        
        # Log for debugging
        self.get_logger().info(
            f'Err: x={horizontal_error:.1f}px, area={object_area:.1f} | '
            f'Cmd: angular.z={angular_output:.3f}, linear.x={linear_output:.3f}',
            throttle_duration_sec=0.5
        )
    
    def compute_angular_control(self, error, current_time):
        """
        PID controller for angular velocity (turning)
        Error: horizontal pixel offset (positive = object on right)
        Output: angular.z (positive = turn left/CCW)
        """
        
        # Get parameters
        kp = self.get_parameter('angular_kp').value
        ki = self.get_parameter('angular_ki').value
        kd = self.get_parameter('angular_kd').value
        max_output = self.get_parameter('angular_max_output').value
        deadband = self.get_parameter('angular_deadband').value
        
        # Deadband - if error is small, don't turn
        if abs(error) < deadband:
            return 0.0
        
        # Initialize on first run
        if self.angular_last_error is None:
            self.angular_last_error = error
            self.angular_last_time = current_time
            return 0.0
        
        # Calculate dt
        dt = current_time - self.angular_last_time
        if dt < 0.001:  # Avoid division by zero
            return 0.0
        
        # Proportional term
        # Note: We negate error so positive error (right) gives negative angular.z (turn right)
        p = -kp * error
        
        # Integral term with anti-windup
        self.angular_error_sum += error * dt
        self.angular_error_sum = max(-1000, min(1000, self.angular_error_sum))
        i = -ki * self.angular_error_sum
        
        # Derivative term
        d = -kd * (error - self.angular_last_error) / dt
        
        # Total output
        output = p + i + d
        
        # Limit output
        output = max(-max_output, min(max_output, output))
        
        # Update state
        self.angular_last_error = error
        self.angular_last_time = current_time
        
        return output
    
    def compute_linear_control(self, object_area, current_time):
        """
        PID controller for linear velocity (forward/backward)
        Error: difference from target area
        Output: linear.x (positive = forward)
        """
        
        # Get parameters
        kp = self.get_parameter('linear_kp').value
        ki = self.get_parameter('linear_ki').value
        kd = self.get_parameter('linear_kd').value
        max_output = self.get_parameter('linear_max_output').value
        deadband = self.get_parameter('linear_deadband').value
        target_area = self.get_parameter('target_object_area').value
        
        # Calculate error (target - actual)
        # Positive error = object too small = move forward
        # Negative error = object too large = move backward
        error = target_area - object_area
        
        # Deadband
        if abs(error) < deadband:
            return 0.0
        
        # Initialize on first run
        if self.linear_last_error is None:
            self.linear_last_error = error
            self.linear_last_time = current_time
            return 0.0
        
        # Calculate dt
        dt = current_time - self.linear_last_time
        if dt < 0.001:
            return 0.0
        
        # Proportional term
        p = kp * error
        
        # Integral term with anti-windup
        self.linear_error_sum += error * dt
        self.linear_error_sum = max(-10000, min(10000, self.linear_error_sum))
        i = ki * self.linear_error_sum
        
        # Derivative term
        d = kd * (error - self.linear_last_error) / dt
        
        # Total output
        output = p + i + d
        
        # Limit output
        output = max(-max_output, min(max_output, output))
        
        # Update state
        self.linear_last_error = error
        self.linear_last_time = current_time
        
        return output

def main(args=None):
    rclpy.init(args=args)
    controller = DualPIDController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
