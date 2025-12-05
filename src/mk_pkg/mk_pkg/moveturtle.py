#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleSquare(Node):
    def __init__(self):
        super().__init__('turtle_square')

        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('speed', 1.0)
        self.declare_parameter('side_len', 4.0)
        self.declare_parameter('turn_speed', 1.0)
        self.declare_parameter('sides', 4)
        self.declare_parameter('loops', 0)                 # 0 = infinite
        self.declare_parameter('pause_sec_between', 0.5)   # pause between shapes

        rate_hz    = float(self.get_parameter('rate_hz').value)
        speed      = float(self.get_parameter('speed').value)
        side_len   = float(self.get_parameter('side_len').value)
        turn_speed = float(self.get_parameter('turn_speed').value)
        sides      = int(self.get_parameter('sides').value)
        self.max_loops = int(self.get_parameter('loops').value)
        self.pause_sec_between = float(self.get_parameter('pause_sec_between').value)

        rate_hz = max(rate_hz, 0.1)
        speed = max(speed, 1e-6)
        turn_speed = max(turn_speed, 1e-6)
        sides = max(sides, 1)

        # Publish autonomy commands here (FSM subscribes to this)
        self.publisher_ = self.create_publisher(Twist, '/mk/autonomy/cmd_vel', 10)

        self._dt = 1.0 / rate_hz
        self.beats_drive = max(1, int(round((side_len / speed) / self._dt)))
        self.beats_turn  = max(1, int(round(((math.pi / 2.0) / turn_speed) / self._dt)))
        self.beats_pause = max(1, int(round(self.pause_sec_between / self._dt)))

        self._forward_cmd = Twist(); self._forward_cmd.linear.x = speed
        self._turn_cmd    = Twist(); self._turn_cmd.angular.z   = turn_speed
        self._stop_cmd    = Twist() 

        self.total_sides = sides
        self.side_index = 0                 
        self.phase = 'drive'              
        self.beat_in_phase = 0
        self.completed_loops = 0

        self.get_logger().info(
            f"Looping {sides}-sided shape; rate={rate_hz:.1f}Hz; "
            f"drive_beats={self.beats_drive}, turn_beats={self.beats_turn}, "
            f"pause={self.pause_sec_between:.2f}s; loops={'∞' if self.max_loops==0 else self.max_loops}"
        )

        self.timer = self.create_timer(self._dt, self.timer_callback)

    def publish(self, msg: Twist):
        self.publisher_.publish(msg)

    # --- Timer beat (the “conductor”) ---
    def timer_callback(self):
        if self.phase == 'drive':
            self.publish(self._forward_cmd)
            self.beat_in_phase += 1
            if self.beat_in_phase >= self.beats_drive:
                self.phase = 'turn'
                self.beat_in_phase = 0
                return

        elif self.phase == 'turn':
            self.publish(self._turn_cmd)
            self.beat_in_phase += 1
            if self.beat_in_phase >= self.beats_turn:
                self.side_index += 1
                self.beat_in_phase = 0
                if self.side_index >= self.total_sides:
                    # finished a polygon — pause, then either loop or stop
                    self.completed_loops += 1
                    self.side_index = 0
                    if self.max_loops and self.completed_loops >= self.max_loops:
                        # final stop and cancel timer
                        self.publish(self._stop_cmd)
                        self.timer.cancel()
                        self.get_logger().info('Completed requested loops; stopping.')
                        return
                    self.phase = 'pause' if self.beats_pause > 0 else 'drive'
                else:
                    self.phase = 'drive'

        elif self.phase == 'pause':
            # publish stop for a short pause between loops
            self.publish(self._stop_cmd)
            self.beat_in_phase += 1
            if self.beat_in_phase >= self.beats_pause:
                self.beat_in_phase = 0
                self.phase = 'drive'

def main(args=None):
    rclpy.init(args=args)
    node = TurtleSquare()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publisher_.publish(Twist())  # stop once on shutdown
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
