import sys
import os
import time
import math
import struct
import threading
import io
from typing import Optional

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
try:
    from robstride_dynamics.bus import RobstrideBus, Motor
    from robstride_dynamics.protocol import ParameterType, CommunicationType
except ImportError:
    from bus import RobstrideBus, Motor
    from protocol import ParameterType, CommunicationType

class PositionControllerMIT:
    def __init__(self, motor_id: int, channel='can0'):
        self.motor_id = motor_id
        self.motor_name = f"motor_{motor_id}"
        self.channel = channel
        self.bus: Optional[RobstrideBus] = None
        self.lock = threading.Lock()
        self.running = True
        self.connected = False
        self.target_position = 0.0
        self.current_pos_rad = 0.0
        self.current_vel_rad = 0.0
        self.kp = 20.0 
        self.kd = 0.3

    def _set_mode_raw(self, mode: int):
        # mode 0 = MIT, 1 = Position, 2 = Speed...
        param_id = ParameterType.MODE[0]
        value_buffer = struct.pack("<bBH", mode, 0, 0)
        data = struct.pack("<HH", param_id, 0x00) + value_buffer
        self.bus.transmit(CommunicationType.WRITE_PARAMETER, self.bus.host_id, self.motor_id, data)
        time.sleep(0.2) # 待機時間を増やす

    def connect(self):
        self.running = True
        motors = {self.motor_name: Motor(id=self.motor_id, model="rs-03")}
        calibration = {self.motor_name: {"direction": 1, "homing_offset": 0.0}}
        
        try:
            self.bus = RobstrideBus(self.channel, motors, calibration)
            self.bus.connect(handshake=True)
            
            with self.lock:
                self.bus.enable(self.motor_name)
                time.sleep(0.5) # モーターの起動を待つ
                self._set_mode_raw(0) # MIT Mode
                
                status = self.bus.read_operation_frame(self.motor_name)
                if status:
                    self.current_pos_rad = status[0]
                    self.target_position = status[0]
            
            self.connected = True
            self.control_thread = threading.Thread(target=self.loop, daemon=True)
            self.control_thread.start()
            return True
        except Exception:
            self.cleanup_bus()
            return False

    def cleanup_bus(self):
        self.connected = False
        self.bus = None

    def loop(self):
        while self.running and self.connected:
            try:
                with self.lock:
                    if self.bus and self.bus.channel_handler:
                        self.bus.write_operation_frame(self.motor_name, self.target_position, self.kp, self.kd, 0.0, 0.0)
                        status = self.bus.read_operation_frame(self.motor_name)
                        if status:
                            self.current_pos_rad = status[0]
                            self.current_vel_rad = status[1]
                time.sleep(0.01)
            except Exception:
                self.connected = False
                break

    def stop_and_exit(self):
        self.running = False
        if self.bus and self.connected:
            with self.lock:
                try:
                    self.bus.disable(self.motor_name)
                except: pass
        self.cleanup_bus()