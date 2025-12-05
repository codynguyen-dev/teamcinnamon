#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float32

_METERS_TO_FEET = 3.28084
_SMOOT_IN_METERS = 1.7018

class DistanceConverter(Node):
    def __init__(self):
        super().__init__('converter')

        self.declare_parameter('output_unit', 'smoots')
        self.add_on_set_parameters_callback(self._on_param_update)

        self._sub = self.create_subscription(Float32, 'distance', self._on_distance, 10)
        self._pub = self.create_publisher(Float32, 'distance_converted', 10)

        self.get_logger().info(
            'node=converter status=ready sub=distance pub=distance_converted param=output_unit'
        )

    def _on_param_update(self, params):
        for p in params:
            if p.name == 'output_unit':
                if p.type_ == p.Type.STRING and p.value.lower() in ('meters', 'feet', 'smoots'):
                    self.get_logger().info(f"output_unit set to {p.value.lower()}")
                    return SetParametersResult(successful=True)
                else:
                    self.get_logger().error('Invalid output_unit. Use one of: meters, feet, smoots')
                    return SetParametersResult(successful=False)
        return SetParametersResult(successful=True)

    def _on_distance(self, msg: Float32):
        unit = self.get_parameter('output_unit').get_parameter_value().string_value.lower()
        meters = float(msg.data)

        if unit == 'meters':
            out = meters
        elif unit == 'feet':
            out = meters * _METERS_TO_FEET
        elif unit == 'smoots':
            out = meters / _SMOOT_IN_METERS
        else:
            out = meters

        self._pub.publish(Float32(data=float(out)))


def main(args=None):
    rclpy.init(args=args)
    node = DistanceConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
