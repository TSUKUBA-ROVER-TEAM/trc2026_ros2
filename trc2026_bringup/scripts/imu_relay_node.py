#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import numpy as np

class ImuRelayNode(Node):
    def __init__(self):
        super().__init__('imu_relay_node')
        
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data_raw_orig',
            self.listener_callback,
            10)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.publisher = self.create_publisher(Imu, '/imu/data_raw', 10)
        
        self.declare_parameter('calibration_count', 100)
        self.declare_parameter('invert_z', True)
        self.declare_parameter('gravity_scale', 9.80665 / 11.09) # 暫定スケール
        
        self.calibration_count = self.get_parameter('calibration_count').value
        self.invert_z = self.get_parameter('invert_z').value
        self.gravity_scale = self.get_parameter('gravity_scale').value
        
        self.gyro_bias = np.array([0.0, 0.0, 0.0])
        self.samples_collected = 0
        self.is_calibrated = False
        self.is_stationary = False
        
        self.get_logger().info('IMU Relay Node started. Calibrating...')

    def odom_callback(self, msg):
        # 非常に小さい閾値で静止判定
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vth = msg.twist.twist.angular.z
        self.is_stationary = (abs(vx) < 1e-6 and abs(vy) < 1e-6 and abs(vth) < 1e-6)

    def listener_callback(self, msg):
        if not self.is_calibrated:
            self.gyro_bias[0] += msg.angular_velocity.x
            self.gyro_bias[1] += msg.angular_velocity.y
            self.gyro_bias[2] += msg.angular_velocity.z
            self.samples_collected += 1
            
            if self.samples_collected >= self.calibration_count:
                self.gyro_bias /= self.samples_collected
                self.is_calibrated = True
                self.get_logger().info(f'Calibration finished. Bias: {self.gyro_bias}')
            return

        new_msg = msg
        # バイアス除去
        new_msg.angular_velocity.x -= self.gyro_bias[0]
        new_msg.angular_velocity.y -= self.gyro_bias[1]
        new_msg.angular_velocity.z -= self.gyro_bias[2]
        
        # 静止時は角速度を強制的に0にする（方位ドリフト防止）
        if self.is_stationary:
            new_msg.angular_velocity.z = 0.0
        
        # 加速度の補正 (特にZ軸の反転とスケール)
        if self.invert_z:
            new_msg.linear_acceleration.z *= -1.0
            
        new_msg.linear_acceleration.x *= self.gravity_scale
        new_msg.linear_acceleration.y *= self.gravity_scale
        new_msg.linear_acceleration.z *= self.gravity_scale
        
        self.publisher.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    imu_relay_node = ImuRelayNode()
    rclpy.spin(imu_relay_node)
    imu_relay_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
