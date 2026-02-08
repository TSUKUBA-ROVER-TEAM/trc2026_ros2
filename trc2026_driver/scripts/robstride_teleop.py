#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from control_msgs.msg import JointJog
import sys, termios, tty

msg = """
J2 & J3 Teleop
---------------------------
J2操作: 'w'(正) / 's'(逆)
J3操作: 'e'(正) / 'd'(逆)
停止:   'k'
終了:   'q'
---------------------------
"""

class TeleopJ2J3(Node):
    def __init__(self):
        super().__init__('robstride_teleop_node')
        self.pub = self.create_publisher(JointJog, '/servo_node/delta_joint_cmds', 10)

    def get_key(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def run(self):
        print(msg)
        while rclpy.ok():
            key = self.get_key()
            out = JointJog()
            out.header.stamp = self.get_clock().now().to_msg()
            
            if key == 'w': out.joint_names, out.velocities = ['arm_2_joint'], [0.3]
            elif key == 's': out.joint_names, out.velocities = ['arm_2_joint'], [-0.3]
            elif key == 'e': out.joint_names, out.velocities = ['arm_3_joint'], [0.3]
            elif key == 'd': out.joint_names, out.velocities = ['arm_3_joint'], [-0.3]
            elif key == 'k': out.joint_names, out.velocities = ['arm_2_joint', 'arm_3_joint'], [0.0, 0.0]
            elif key == 'q': break
            else: continue
            
            self.pub.publish(out)

def main():
    rclpy.init()
    TeleopJ2J3().run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()