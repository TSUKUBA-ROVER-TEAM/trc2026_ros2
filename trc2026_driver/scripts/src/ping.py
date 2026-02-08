#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RobStride CAN 总线扫描工具 (Python 版)

用法: python3 scan_bus.py [channel]
示例: 
    sudo python3 scan_bus.py can0
    sudo python3 scan_bus.py can1

此脚本将 ping 1 到 254 范围内的所有 ID，并报告响应的电机。
**注意：** 访问 CAN 硬件通常需要 'sudo' 权限。
"""

import sys
import os
import time

# --- 导入 SDK ---
# 假设此脚本与 position_control_mit.py 在同一目录
# (即，上一级目录是 SDK 的根目录)
try:
    # 尝试将 SDK 根目录添加到路径
    sdk_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if sdk_path not in sys.path:
        sys.path.insert(0, sdk_path)
    
    # 1. 尝试从已安装的包导入
    from robstride_dynamics import RobstrideBus
except ImportError:
    # 2. 尝试从本地文件导入 (如果 SDK 未安装)
    try:
        print("未找到 'robstride_dynamics' 包, 尝试从本地文件导入...")
        from bus import RobstrideBus
    except ImportError as e:
        print(f"❌ 无法导入 RobstrideBus SDK: {e}")
        print("请确保此脚本的上一级目录是 SDK 的根目录,")
        print("或者 SDK 已经通过 'pip install -e .' 安装。")
        sys.exit(1)

def main():
    # --- 1. 获取 CAN 通道 ---
    if len(sys.argv) > 1:
        channel = sys.argv[1]
    else:
        channel = "can0"

    print(f"🚀 RobStride 总线扫描工具")
    print(f"📡 正在扫描通道: {channel}")
    print(f"🔍 搜索范围: ID 1 到 254")
    print("...")
    time.sleep(1) # 暂停一下让用户阅读

    # --- 2. 运行扫描 ---
    found_motors = None
    try:
        # RobstrideBus.scan_channel 已经为我们实现了所有逻辑
        # 它内部使用了 tqdm 来显示进度条
        found_motors = RobstrideBus.scan_channel(channel, start_id=1, end_id=255) # end_id=255 会扫描到 254
    
    except Exception as e:
        print(f"\n❌ 扫描出错: {e}")
        if "Operation not permitted" in str(e) or "Permission denied" in str(e):
            print("🔑 权限错误：请使用 'sudo' 运行此脚本来访问 CAN 硬件。")
            print(f"   示例: sudo python3 {sys.argv[0]} {channel}")
        elif "No such device" in str(e):
            print(f"🔌 设备错误：找不到 CAN 接口 '{channel}'。")
        sys.exit(1)

    # --- 3. 打印结果 ---
    if not found_motors:
        print("\n🚫 未在总线上找到任何响应的电机。")
    else:
        print("\n✅ 扫描完成！发现以下电机：")
        print("=" * 60)
        print(f"{'电机 ID':<10} | {'MCU 唯一标识符 (UUID)':<45}")
        print("-" * 60)
        
        # found_motors 是一个字典: {id: (id, uuid_bytearray)}
        # 我们按 ID 排序
        for motor_id in sorted(found_motors.keys()):
            # value 是一个元组 (id, uuid)
            _id, uuid = found_motors[motor_id]
            
            # 将 bytearray 转换为更易读的十六进制字符串
            uuid_hex = uuid.hex() # 'hex()' 是 bytearray 的一个方法
            
            print(f"{motor_id:<10} | {uuid_hex}")
            
        print("=" * 60)

if __name__ == "__main__":
    main()