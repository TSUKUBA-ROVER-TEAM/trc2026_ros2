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
    from robstride_dynamics import RobstrideBus, Motor, ParameterType, CommunicationType
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
        self.kp = 30.0
        self.kd = 0.5

    def _set_mode_raw(self, mode: int):
        device_id = self.bus.motors[self.motor_name].id
        param_id, _, _ = ParameterType.MODE
        value_buffer = struct.pack("<bBH", mode, 0, 0)
        data = struct.pack("<HH", param_id, 0x00) + value_buffer
        self.bus.transmit(CommunicationType.WRITE_PARAMETER, self.bus.host_id, device_id, data)
        time.sleep(0.1)

    def connect(self):
        self.cleanup_bus()
        self.running = True
        motors = {self.motor_name: Motor(id=self.motor_id, model="rs-03")}
        calibration = {self.motor_name: {"direction": 1, "homing_offset": 0.0}}
        
        original_stdout = sys.stdout
        sys.stdout = io.StringIO()
        
        try:
            self.bus = RobstrideBus(self.channel, motors, calibration)
            self.bus.connect(handshake=True)
            with self.lock:
                self.bus.enable(self.motor_name)
                time.sleep(0.2)
                self._set_mode_raw(0)
                self.target_position = 0.0
                self.bus.write_operation_frame(self.motor_name, self.target_position, self.kp, self.kd, 0.0, 0.0)
                status = self.bus.read_operation_frame(self.motor_name)
                if status:
                    self.current_pos_rad = status[0]
                    self.current_vel_rad = status[1]
            
            sys.stdout = original_stdout
            self.connected = True
            self.control_thread = threading.Thread(target=self.loop, daemon=True)
            self.control_thread.start()
            return True
        except Exception:
            sys.stdout = original_stdout
            self.cleanup_bus()
            return False

    def cleanup_bus(self):
        self.connected = False
        if self.bus:
            if hasattr(self.bus, 'channel_handler'):
                self.bus.channel_handler = None
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
                    else:
                        break
                time.sleep(0.02)
            except Exception:
                self.connected = False
                break

    def set_angle(self, angle_degrees: float):
        self.target_position = math.radians(max(-720.0, min(720.0, angle_degrees)))

    def stop_and_exit(self):
        self.running = False
        if self.bus and self.connected:
            with self.lock:
                try:
                    self.bus.write_operation_frame(self.motor_name, 0.0, self.kp, self.kd, 0.0, 0.0)
                    time.sleep(0.1)
                    self.bus.disable(self.motor_name)
                except:
                    pass
        self.cleanup_bus()