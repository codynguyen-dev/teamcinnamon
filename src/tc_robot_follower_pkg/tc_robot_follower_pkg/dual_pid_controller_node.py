#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

class DualPIDController(Node):
    def __init__(self):
        super().__init__('dual_pid_controller')

        # angular tuning
        self.declare_parameter('angular_kp', 0.005)
        self.declare_parameter('angular_ki', 0.0)
        self.declare_parameter('angular_kd', 0.001)
        self.declare_parameter('angular_max_output', 1.0)

        # linear distance
        self.declare_parameter('linear_kp', 0.0003)
        self.declare_parameter('linear_ki', 0.0)
        self.declare_parameter('linear_kd', 0.0)
        self.declare_parameter('linear_max_output', 0.5)

        # target object size
        self.declare_parameter('target_object_area', 40000.0)

        # deadband, don't move if error is small
        self.declare_parameter('angular_deadband', 10.0)  # pixels
        self.declare_parameter('linear_deadband', 500.0)  # area pixels

        # angular controller state
        self.angular_error_sum = 0.0
        self.angular_last_error = None
        self.angular_last_time = None

        # linear controller state
        self.linear_error_sum = 0.0
        self.linear_last_error = None
        self.linear_last_time = None

        self.measurements_sub = self.create_subscription(
            Float32MultiArray,
            'object_measurements',
            self.measurements_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            'autonomous_cmd', # autonomous command output
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

        horizontal_error = msg.data[0]  # positive right, negative left
        object_area = msg.data[1]
        detected = msg.data[2]

        # stop if no object detected
        if detected == 0.0:
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)

            self.angular_error_sum = 0.0
            self.linear_error_sum = 0.0
            self.angular_last_error = None
            self.linear_last_error = None
            return

        current_time = self.get_clock().now().nanoseconds / 1e9

        # angular control
        angular_output = self.compute_angular_control(horizontal_error, current_time)

        # linear control
        linear_output = self.compute_linear_control(object_area, current_time)

        # publish Twist message
        cmd = Twist()
        cmd.angular.z = angular_output
        cmd.linear.x = linear_output

        self.cmd_pub.publish(cmd)

        # log for debugging
        self.get_logger().info(
            f'Err: x={horizontal_error:.1f}px, area={object_area:.1f} | '
            f'Cmd: angular.z={angular_output:.3f}, linear.x={linear_output:.3f}',
            throttle_duration_sec=0.5
        )

    def compute_angular_control(self, error, current_time):

        # get parameters
        kp = self.get_parameter('angular_kp').value
        ki = self.get_parameter('angular_ki').value
        kd = self.get_parameter('angular_kd').value
        max_output = self.get_parameter('angular_max_output').value
        deadband = self.get_parameter('angular_deadband').value

        if abs(error) < deadband:
            return 0.0

        if self.angular_last_error is None:
            self.angular_last_error = error
            self.angular_last_time = current_time
            return 0.0

        # calculate dt
        dt = current_time - self.angular_last_time
        if dt < 0.001:
            return 0.0

        # proportional term
        p = -kp * error

        # integral term
        self.angular_error_sum += error * dt
        self.angular_error_sum = max(-1000, min(1000, self.angular_error_sum))
        i = -ki * self.angular_error_sum

        d = -kd * (error - self.angular_last_error) / dt

        # total output
        output = p + i + d

        output = max(-max_output, min(max_output, output))

        # update state
        self.angular_last_error = error
        self.angular_last_time = current_time

        return output

    def compute_linear_control(self, object_area, current_time):

        # get parameters
        kp = self.get_parameter('linear_kp').value
        ki = self.get_parameter('linear_ki').value
        kd = self.get_parameter('linear_kd').value
        max_output = self.get_parameter('linear_max_output').value
        deadband = self.get_parameter('linear_deadband').value
        target_area = self.get_parameter('target_object_area').value

        # calculate error
        error = target_area - object_area

        if abs(error) < deadband:
            return 0.0

        # initialize on first run
        if self.linear_last_error is None:
            self.linear_last_error = error
            self.linear_last_time = current_time
            return 0.0

        # calculate dt
        dt = current_time - self.linear_last_time
        if dt < 0.001:
            return 0.0

        # proportional term
        p = kp * error

        # integral term
        self.linear_error_sum += error * dt
        self.linear_error_sum = max(-10000, min(10000, self.linear_error_sum))
        i = ki * self.linear_error_sum

        # derivative term
        d = kd * (error - self.linear_last_error) / dt

        # total output
        output = p + i + d

        output = max(-max_output, min(max_output, output))

        # update state
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