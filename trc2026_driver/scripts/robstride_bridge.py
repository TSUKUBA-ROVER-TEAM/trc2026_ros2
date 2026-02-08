#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_msgs.msg import JointJog
import os
import sys

script_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(script_dir)

try:
    from position_control import PositionControllerMIT
except ImportError:
    from src.position_control import PositionControllerMIT

class RobStrideBridge(Node):
    def __init__(self):
        super().__init__('robstride_bridge_node')
        self.target_motor_id = 2
        self.controller = PositionControllerMIT(self.target_motor_id)

        self.arm_joints = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint']
        self.other_joints = [
            'hand_right_joint', 'steer_left_backward_joint', 'drive_left_backward_joint',
            'steer_left_forward_joint', 'drive_left_forward_joint', 'steer_right_backward_joint',
            'drive_right_backward_joint', 'steer_right_forward_joint', 'drive_right_forward_joint'
        ]

        self.js_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.sub = self.create_subscription(JointJog, '/servo_node/delta_joint_cmds', self.servo_cmd_callback, 10)

        self.dt = 0.01
        self.timer = self.create_timer(self.dt, self.update_status)
        self.conn_timer = self.create_timer(2.0, self.attempt_connection)

    def attempt_connection(self):
        if not self.controller.connected:
            self.controller.connect()

    def update_status(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        all_names = self.arm_joints + self.other_joints
        msg.name = all_names
        msg.position = [0.0] * len(all_names)
        msg.velocity = [0.0] * len(all_names)
        msg.effort = [0.0] * len(all_names)

        if self.controller.connected:
            msg.position[0] = float(self.controller.current_pos_rad)
            msg.velocity[0] = float(self.controller.current_vel_rad)

        self.js_pub.publish(msg)

    def servo_cmd_callback(self, msg):
        if self.controller.connected and 'arm_1_joint' in msg.joint_names:
            idx = msg.joint_names.index('arm_1_joint')
            self.controller.target_position += msg.velocities[idx] * self.dt

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
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()