#!/usr/bin/env python3

import threading
import time

from trc2026_msgs.msg import Can
import can

import rclpy
from rclpy.node import Node


class CanRx(Node):

    def __init__(self):
        super().__init__('can_rx_node')

        self.can_interface = 'can0'
        self.bus = None
        self.running = True

        self.publisher = self.create_publisher(Can, 'from_can_bus_fd', 10)

        # 再接続管理用のタイマー
        self.create_timer(1.0, self.try_connect)

        # 受信専用スレッドの開始
        self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self.rx_thread.start()

        self.get_logger().info('CanRx Node Started. Waiting for connection...')

    def try_connect(self):
        if self.bus is not None:
            return

        try:
            self.bus = can.Bus(
                interface='socketcan',
                channel=self.can_interface,
                fd=False
            )
            self.get_logger().info(f'Rx: Connected to {self.can_interface}')
        except Exception:
            self.bus = None

    def rx_loop(self):
        while self.running:
            if self.bus is None:
                # 接続がなければ少し待機してループを回す
                time.sleep(0.1)
                continue

            try:
                msg = self.bus.recv(timeout=0.5)

                if msg is not None:
                    ros_msg = self._convert_can_to_ros(msg)
                    self.publisher.publish(ros_msg)

            except (can.CanOperationError, OSError) as e:
                self.get_logger().error(
                    f'Rx Error: {e}. Resetting connection...')
                self.cleanup_bus()
            except Exception as e:
                self.get_logger().error(f'Rx Unexpected loop error: {e}')
                self.cleanup_bus()

    def cleanup_bus(self):
        if self.bus:
            try:
                self.bus.shutdown()
            except Exception as e:
                # Ignore shutdown errors but log them for diagnostics
                self.get_logger().warning(
                    f'Error while shutting down CAN bus: {e}')
        self.bus = None

    def _convert_can_to_ros(self, can_msg: can.Message) -> Can:
        ros_msg = Can()
        ros_msg.header.stamp = self.get_clock().now().to_msg()
        ros_msg.header.frame_id = self.can_interface
        ros_msg.id = can_msg.arbitration_id
        ros_msg.is_extended = can_msg.is_extended_id
        ros_msg.is_error = can_msg.is_error_frame
        ros_msg.len = can_msg.dlc
        ros_msg.data = list(can_msg.data)
        return ros_msg

    def stop(self):
        self.running = False
        self.cleanup_bus()
        if self.rx_thread.is_alive():
            self.rx_thread.join(timeout=1.0)


def main():
    rclpy.init()
    node = CanRx()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
