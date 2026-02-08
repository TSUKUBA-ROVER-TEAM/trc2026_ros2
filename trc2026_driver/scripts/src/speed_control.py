#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RobStride 速度模式控制脚本 (修复版)
模式: Mode 2 (Speed Control Mode)
通信: 使用 WRITE_PARAMETER 更新 VELOCITY_TARGET (spd_ref)

用法: python3 speed_control.py <motor_id>
"""

import sys
import os
import time
import threading
import signal
from typing import Optional

# 尝试导入 SDK
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
try:
    from robstride_dynamics import RobstrideBus, Motor, ParameterType
except ImportError:
    # 假设当前目录结构
    try:
        from bus import RobstrideBus, Motor
        from protocol import ParameterType
    except ImportError as e:
        print(f"❌ 无法导入 SDK: {e}")
        sys.exit(1)

class SpeedController:
    def __init__(self, motor_id: int, channel='can0'):
        self.motor_id = motor_id
        self.motor_name = f"motor_{motor_id}"
        self.channel = channel
        
        self.bus: Optional[RobstrideBus] = None
        self.lock = threading.Lock()  # 互斥锁，防止Socket冲突
        
        self.running = True
        self.connected = False
        self.target_velocity = 0.0
        self.current_status = None
        
        # 默认参数
        self.max_velocity = 20.0  # rad/s 安全限制
        self.kp = 2.0
        self.ki = 0.5

    def _signal_handler(self, signum, frame):
        self.stop_and_exit()

    def connect(self):
        print(f"🔍 正在连接 CAN 通道 {self.channel}...")
        
        # 定义电机
        motors = {
            self.motor_name: Motor(id=self.motor_id, model="rs-06") # 模型型号请根据实际修改
        }
        
        # 简单的校准参数
        calibration = {
            self.motor_name: {"direction": 1, "homing_offset": 0.0}
        }

        try:
            self.bus = RobstrideBus(self.channel, motors, calibration)
            self.bus.connect(handshake=True)
            
            # 激活电机
            print(f"⚡ 激活电机 ID: {self.motor_id} ...")
            self.bus.enable(self.motor_name)
            time.sleep(0.5)

            # 设置为速度模式 (Mode 2)
            print("⚙️ 设置为速度控制模式 (Mode 2)...")
            self.bus.write(self.motor_name, ParameterType.MODE, 2)
            
            # 初始化 PID 和限制
            print("⚙️ 写入控制参数...")
            self.bus.write(self.motor_name, ParameterType.VELOCITY_LIMIT, self.max_velocity)
            self.bus.write(self.motor_name, ParameterType.VELOCITY_KP, self.kp)
            self.bus.write(self.motor_name, ParameterType.VELOCITY_KI, self.ki)
            
            # 归零目标
            self.bus.write(self.motor_name, ParameterType.VELOCITY_TARGET, 0.0)
            
            self.connected = True
            print("✅ 初始化完成！")
            return True
            
        except Exception as e:
            print(f"❌ 连接失败: {e}")
            return False

    def loop(self):
        """控制线程：持续发送心跳/速度指令并读取状态"""
        print("🔄 控制循环已启动")
        
        while self.running and self.connected:
            try:
                with self.lock:
                    # 在 Mode 2 下，我们需要写入 VELOCITY_TARGET
                    # bus.write 会等待回包 (receive_status_frame)，所以这本身就是一种状态读取
                    # Protocol 0x700A = VELOCITY_TARGET
                    
                    self.bus.write(self.motor_name, ParameterType.VELOCITY_TARGET, self.target_velocity)
                    
                    # 如果想读取更详细的状态（如当前扭矩），可以使用 read_operation_frame
                    # 但 write 的回包里其实已经包含 status 数据了，SDK 的 write 内部调用了 receive_status_frame
                    # 这里我们不做额外的读取以保持高频率
                    
                time.sleep(0.05) # 20Hz 刷新率，防止总线拥堵
                
            except Exception as e:
                print(f"⚠️ 通信错误: {e}")
                time.sleep(0.5)

    def set_velocity(self, vel: float):
        """设置目标速度（带限幅）"""
        vel = max(-self.max_velocity, min(self.max_velocity, vel))
        self.target_velocity = vel
        print(f" -> 目标设定: {self.target_velocity:.2f} rad/s")

    def stop_and_exit(self):
        print("\n🛑 正在停止...")
        self.running = False
        self.target_velocity = 0.0
        
        if self.bus and self.connected:
            try:
                with self.lock:
                    # 先停
                    self.bus.write(self.motor_name, ParameterType.VELOCITY_TARGET, 0.0)
                    time.sleep(0.2)
                    # 禁用
                    self.bus.disable(self.motor_name)
            except Exception:
                pass
            self.bus.disconnect()
        sys.exit(0)

    def run_interactive(self):
        # 启动后台发送线程
        t = threading.Thread(target=self.loop, daemon=True)
        t.start()

        print("\n" + "="*40)
        print(f"🎮 速度控制台 (ID: {self.motor_id})")
        print("="*40)
        print("👉 直接输入数字 (rad/s) 回车即可改变速度")
        print("👉 输入 '0' 停止")
        print("👉 输入 'q' 退出")
        print(f"⚠️  当前安全限速: ±{self.max_velocity} rad/s")
        print("-" * 40)

        while True:
            try:
                cmd = input(f"[{self.target_velocity:.1f} rad/s] >> ").strip().lower()
                
                if not cmd:
                    continue
                    
                if cmd in ['q', 'quit', 'exit']:
                    break
                
                try:
                    vel = float(cmd)
                    self.set_velocity(vel)
                except ValueError:
                    print("❌ 无效输入，请输入数字")

            except KeyboardInterrupt:
                break
        
        self.stop_and_exit()

def main():
    if len(sys.argv) < 2:
        print("用法: python3 speed_control.py <motor_id>")
        sys.exit(1)
        
    motor_id = int(sys.argv[1])
    
    controller = SpeedController(motor_id)
    signal.signal(signal.SIGINT, controller._signal_handler)
    
    if controller.connect():
        controller.run_interactive()

if __name__ == "__main__":
    main()