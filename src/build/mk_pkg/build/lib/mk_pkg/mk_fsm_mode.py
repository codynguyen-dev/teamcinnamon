#!/usr/bin/env python3
import enum
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger

class Mode(enum.Enum):
    AUTONOMOUS     = 'AUTO'
    TELEOPERATED   = 'TELEOP'
    OBSTACLE_STOP  = 'OBS'
    ESTOP          = 'ESTOP'   # latched

class ModeMuxFSM(Node):

    def __init__(self):
        super().__init__('mk_mode_fsm')

        self.declare_parameter('rate_hz', 20.0)
        rate_hz = float(self.get_parameter('rate_hz').value)
        rate_hz = max(1.0, rate_hz)

        self.pub_mux  = self.create_publisher(Twist, '/cmd_vel_mux', 10)
        self.sub_tel  = self.create_subscription(Twist, '/turtle1/cmd_vel', self._on_tel, 10)
        self.sub_auto = self.create_subscription(Twist, '/mk/autonomy/cmd_vel', self._on_auto, 10)
        self.sub_req  = self.create_subscription(String, '/mk/mode_request', self._on_mode_request, 10)
        self.sub_rst  = self.create_subscription(Bool, '/mk/estop_reset', self._on_estop_reset, 10)

        self.srv_status = self.create_service(Trigger, 'mk_estop_status', self._srv_status)

        self._requested: Mode = Mode.AUTONOMOUS
        self._selected: Mode  = Mode.AUTONOMOUS
        self._estop_latched: bool = False

        self._last_tel: Optional[Twist]  = None
        self._last_auto: Optional[Twist] = None
        self._zero = Twist()

        self.timer = self.create_timer(1.0 / rate_hz, self._tick)

        self.get_logger().info('mk_mode_fsm ready. Modes: AUTO, TELEOP, OBS, ESTOP.')

    # --- Subscribers ---------------------------------------------------------

    def _on_tel(self, msg: Twist):
        self._last_tel = msg

    def _on_auto(self, msg: Twist):
        self._last_auto = msg

    def _on_mode_request(self, msg: String):
        token = msg.data.strip().upper()
        try:
            self._requested = Mode(token)
        except Exception:
            self.get_logger().warn(f'Unknown mode request "{msg.data}". Use AUTO | TELEOP | OBS | ESTOP.')
            return
        self._apply_transition(event=f'request:{self._requested.value}')

        self.get_logger().info(
            f"request={self._requested.value}  selected={self._selected.value}  estop_latched={self._estop_latched}"
        )

    def _on_estop_reset(self, msg: Bool):
        if msg.data:
            if self._estop_latched:
                self._estop_latched = False
                self._apply_transition(event='estop_reset')
                self.get_logger().info('ESTOP unlatched.')
            else:
                self.get_logger().info('ESTOP reset received but latch was not set.')

    # --- FSM logic -----------------------------------------------------------

    def _apply_transition(self, event: str):
        # Latching 
        if self._requested is Mode.ESTOP:
            self._estop_latched = True
        if self._estop_latched:
            self._selected = Mode.ESTOP
        elif self._requested is Mode.OBSTACLE_STOP:
            self._selected = Mode.OBSTACLE_STOP
        elif self._requested in (Mode.AUTONOMOUS, Mode.TELEOPERATED):
            self._selected = self._requested
        else:
            # Fallback safety
            self._selected = Mode.OBSTACLE_STOP

    def _tick(self):
        # Decide what to publish
        if self._selected is Mode.ESTOP:
            self.pub_mux.publish(self._zero)
        elif self._selected is Mode.OBSTACLE_STOP:
            self.pub_mux.publish(self._zero)
        elif self._selected is Mode.TELEOPERATED:
            self.pub_mux.publish(self._last_tel if self._last_tel is not None else self._zero)
        else:  
            self.pub_mux.publish(self._last_auto if self._last_auto is not None else self._zero)

    # --- Debug service -------------------------------------------------------

    def _srv_status(self, req, resp):
        resp.success = True
        resp.message = f"selected={self._selected.value}, requested={self._requested.value}, estop_latched={self._estop_latched}"
        return resp


def main():
    rclpy.init()
    node = ModeMuxFSM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
