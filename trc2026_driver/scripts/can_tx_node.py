#!/usr/bin/env python3


from trc2026_msgs.msg import Can
import can

import rclpy
from rclpy.node import Node


class CanTx(Node):

    def __init__(self):
        super().__init__('can_tx_node')
        self.can_interface = 'can0'
        self.bus = None

        # 接続・再接続用のタイマー (1秒おき)
        self.create_timer(1.0, self.try_connect)

        self.subscriber = self.create_subscription(
            Can,
            'to_can_bus_fd',
            self.can_tx_callback,
            10,
        )
        self.get_logger().info('CanTx Node Started. Waiting for connection...')

    def try_connect(self):
        if self.bus is not None:
            return

        try:
            self.bus = can.Bus(
                interface='socketcan',
                channel=self.can_interface,
                fd=False
            )
            self.get_logger().info(f'Tx: Connected to {self.can_interface}')
        except Exception:
            self.bus = None

    def can_tx_callback(self, msg: Can):
        if self.bus is None:
            return

        try:
            # メッセージ構築
            data_bytes = bytes(msg.data[:msg.len])
            can_msg = can.Message(
                arbitration_id=msg.id,
                data=data_bytes,
                is_extended_id=msg.is_extended,
                is_fd=False
            )

            # 送信実行
            self.bus.send(can_msg)

        except (can.CanError, OSError) as e:
            # Buffer full error (ENOBUFS) should be handled without reset
            if isinstance(e, OSError) and e.errno == 105:
                self.get_logger().warning('Tx: No buffer space available, dropping frame.')
                return
            
            # USBが抜けると OSError: [Errno 19] No such device 等が発生
            self.get_logger().error(f'Tx Error: {e}. Resetting connection...')
            self.cleanup_bus()
        except Exception as e:
            self.get_logger().error(f'Tx Unexpected error: {e}')
            self.cleanup_bus()

    def cleanup_bus(self):
        if self.bus:
            try:
                self.bus.shutdown()
            except Exception as e:
                # Ignore shutdown errors but log them for diagnostics
                self.get_logger().warning(f'Error while shutting down CAN bus: {e}')
        self.bus = None

    def __del__(self):
        self.cleanup_bus()


def main():
    rclpy.init()
    node = CanTx()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
