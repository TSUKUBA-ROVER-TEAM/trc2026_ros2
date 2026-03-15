#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField


class MagRelayNode(Node):
    def __init__(self):
        super().__init__('mag_relay_node')

        self.declare_parameter('input_topic', '/imu/mag')
        self.declare_parameter('output_topic', '/imu/mag_calibrated')
        self.declare_parameter('hard_iron_offset', [0.0, 0.0, 0.0])
        self.declare_parameter('soft_iron_matrix', [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ])
        self.declare_parameter('lowpass_alpha', 1.0)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.hard_iron_offset = np.array(self.get_parameter('hard_iron_offset').value, dtype=float)
        self.soft_iron_matrix = np.array(self.get_parameter('soft_iron_matrix').value, dtype=float).reshape(3, 3)
        self.lowpass_alpha = float(self.get_parameter('lowpass_alpha').value)

        self.sub = self.create_subscription(
            MagneticField,
            self.input_topic,
            self.listener_callback,
            10
        )
        self.pub = self.create_publisher(MagneticField, self.output_topic, 10)

        self.prev = None

        self.get_logger().info(
            f'Mag Relay Node started. input={self.input_topic}, output={self.output_topic}, '
            f'alpha={self.lowpass_alpha}'
        )

    def listener_callback(self, msg: MagneticField):
        m = np.array([
            msg.magnetic_field.x,
            msg.magnetic_field.y,
            msg.magnetic_field.z,
        ], dtype=float)

        corrected = self.soft_iron_matrix.dot(m - self.hard_iron_offset)

        if self.lowpass_alpha < 1.0:
            if self.prev is None:
                filtered = corrected
            else:
                a = self.lowpass_alpha
                filtered = a * corrected + (1.0 - a) * self.prev
            self.prev = filtered
        else:
            filtered = corrected

        out = MagneticField()
        out.header = msg.header
        out.magnetic_field.x = float(filtered[0])
        out.magnetic_field.y = float(filtered[1])
        out.magnetic_field.z = float(filtered[2])
        out.magnetic_field_covariance = msg.magnetic_field_covariance

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = MagRelayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
