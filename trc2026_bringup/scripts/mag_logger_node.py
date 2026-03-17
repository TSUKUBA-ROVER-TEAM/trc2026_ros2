#!/usr/bin/env python3
import csv
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField


class MagLoggerNode(Node):
    def __init__(self):
        super().__init__('mag_logger_node')

        self.declare_parameter('input_topic', '/imu/mag')
        self.declare_parameter('output_csv', '/tmp/mag_samples.csv')
        self.declare_parameter('max_samples', 0)
        self.declare_parameter('flush_every', 50)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_csv = self.get_parameter('output_csv').value
        self.max_samples = int(self.get_parameter('max_samples').value)
        self.flush_every = int(self.get_parameter('flush_every').value)

        os.makedirs(os.path.dirname(self.output_csv), exist_ok=True)
        self.csv_file = open(self.output_csv, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(['stamp_sec', 'stamp_nanosec', 'x', 'y', 'z'])

        self.count = 0
        self.sub = self.create_subscription(
            MagneticField,
            self.input_topic,
            self.listener_callback,
            50
        )

        self.get_logger().info(
            f'Mag Logger started. input={self.input_topic}, output={self.output_csv}, '
            f'max_samples={self.max_samples}'
        )

    def listener_callback(self, msg: MagneticField):
        self.writer.writerow([
            msg.header.stamp.sec,
            msg.header.stamp.nanosec,
            msg.magnetic_field.x,
            msg.magnetic_field.y,
            msg.magnetic_field.z,
        ])
        self.count += 1
        self.get_logger().info(f'Logged sample #{self.count}: {msg.magnetic_field}')

        if self.flush_every > 0 and self.count % self.flush_every == 0:
            self.csv_file.flush()

        if self.max_samples > 0 and self.count >= self.max_samples:
            self.csv_file.flush()
            self.csv_file.close()
            self.get_logger().info('Reached max_samples, shutting down.')
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = MagLoggerNode()
    rclpy.spin(node)
    node.destroy_node()
    if not node.csv_file.closed:
        node.csv_file.close()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
