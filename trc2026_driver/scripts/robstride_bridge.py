#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
import os
import sys

script_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(script_dir)

try:
    from robstride_dynamics.src.position_control import PositionControllerMIT
except ImportError:
    from src.position_control import PositionControllerMIT

class RobStrideBridge(Node):
    def __init__(self):
        super().__init__('robstride_bridge_node')
        self.motor_id = 2
        self.controller = PositionControllerMIT(self.motor_id)
        self.js_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.sub = self.create_subscription(JointTrajectory, 'joint_trajectory', self.cmd_callback, 10)
        self.timer = self.create_timer(0.01, self.update_status)
        self.conn_timer = self.create_timer(1.0, self.attempt_connection)
        self.waiting_logged = False

    def attempt_connection(self):
        if not self.controller.connected:
            if not self.waiting_logged:
                self.get_logger().warn('Waiting for motor...')
                self.waiting_logged = True
            
            if self.controller.connect():
                self.get_logger().info('Motor online.')
                self.waiting_logged = False
    
    def update_status(self):
        if not self.controller.connected:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['arm_1_joint']
        msg.position = [float(self.controller.current_pos_rad)]
        msg.velocity = [float(self.controller.current_vel_rad)]
        self.js_pub.publish(msg)

    def cmd_callback(self, msg):
        if self.controller.connected and msg.points:
            self.controller.target_position = msg.points[-1].positions[0]

    def destroy_node(self):
        self.controller.stop_and_exit()
        super().destroy_node()

def main():
    rclpy.init()
    node = RobStrideBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()