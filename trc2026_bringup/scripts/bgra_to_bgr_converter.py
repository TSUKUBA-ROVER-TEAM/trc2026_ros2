#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class BgraToBgrConverter(Node):
    def __init__(self):
        super().__init__('bgra_to_bgr_converter')
        
        # パラメータの宣言
        self.declare_parameter('input_topic', '/rgb/image_raw')
        self.declare_parameter('output_topic', '/rgb/image_bgr')
        
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        
        self.bridge = CvBridge()
        
        # サブスクライバーとパブリッシャー
        self.subscription = self.create_subscription(
            Image,
            input_topic,
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Image, output_topic, 10)
        
        self.get_logger().info(f'Converting {input_topic} (BGRA) to {output_topic} (BGR)')

    def listener_callback(self, msg):
        try:
            # BGRA画像をOpenCV形式に変換
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgra8')
            
            # BGRに変換
            bgr_image = cv2.cvtColor(cv_image, cv2.COLOR_BGRA2BGR)
            
            # ROSメッセージに戻してパブリッシュ
            output_msg = self.bridge.cv2_to_imgmsg(bgr_image, encoding='bgr8')
            output_msg.header = msg.header
            self.publisher.publish(output_msg)
            
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = BgraToBgrConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()