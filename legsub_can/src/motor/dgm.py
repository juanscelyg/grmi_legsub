#!/usr/bin/env python

import numpy as np
import ctypes
import struct
import math

class motor():
    def __init__(self):
        self.angle = 0.0
        self.d_angle = 0.0
        self.speed = 0.0
        self.d_vel = 0.0
        self.current = 0.0
        self.mode = ""
        self.zero = 0.0

class dgm():
    ENABLE = True
    DISABLE = False
    BITS_16 = 65535
    BITS_12 = 4095
    ZERO = -4.71 # Zero reference
    #
    POSITION_CONTROL = "0xFF"
    SPEED_CONTROL = "0xFB"
    POSITION_CONTROL_MIN = -4.0*math.pi
    POSITION_CONTROL_MAX = 4.0*math.pi
    POSITION_CONTROL_LOW_LIMIT = 0
    POSITION_CONTROL_HIGH_LIMIT = BITS_16
    SPEED_CONTROL_MIN = 0.0
    SPEED_CONTROL_MAX = 65.0
    GAIN_KP_MAX = 1000.0
    GAIN_KD_MAX = 5.0
    GAIN_FF_MAX = 33.0
    CURRENT_MIN = -14.0
    CURRENT_MAX = 14.0
    MOTOR_ENTER = "0xFC"
    MOTOR_EXIT = "0xFD"
    SET_ZERO = "0xFE"
    #
    labels={
        "0xFF":"POSITION_CONTROL",
        "0xFB":"SPEED_CONTROL",
        "0xFC":"MOTOR_ENTER",
        "0xFD":"MOTOR_EXIT",
        "0xFE":"SET_ZERO"
    }

class frame():
    def __init__(self, id):
        self.id=ctypes.c_uint8(id).value
        self.real_id = ctypes.c_uint32(self.id).value
        self.is_extended = dgm.DISABLE
        self.dlc = ctypes.c_uint8(8).value
        self.data = []
        self.flush()

    def map(self,  x,  in_min,  in_max,  out_min,  out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    
    def flush(self):
        for i in range(8):
            self.data.append(ctypes.c_uint8(int("0xFF", 16)).value)

class encode():
    def __init__(self):
        pass

    def set_angle(self, id, d_angle, d_vel, kp, kd, ff):
        _frame = frame(id)
        pos = int(_frame.map(d_angle, dgm.POSITION_CONTROL_MIN, dgm.POSITION_CONTROL_MAX, dgm.POSITION_CONTROL_HIGH_LIMIT, dgm.POSITION_CONTROL_LOW_LIMIT))
        vel = int(_frame.map(d_vel, 0, dgm.SPEED_CONTROL_MAX, 0, dgm.BITS_12))
        _kp = int(_frame.map(kp, 0, dgm.GAIN_KP_MAX, 0, dgm.BITS_12)) 
        _kd = int(_frame.map(kd, 0, dgm.GAIN_KD_MAX, 0, dgm.BITS_12)) 
        _ff = int(_frame.map(ff, 0, dgm.GAIN_FF_MAX, 0, dgm.BITS_12)) 
        #print(pos, vel, _kp, _kd, _ff)
        _frame.data[0] = ((pos>>8) & 0xff)
        _frame.data[1] = (pos & 0xff)
        _frame.data[2] = ((vel >> 4) & 0xff)
        _frame.data[3] = (((vel & 0x000f) << 4) + ((_kp >> 8) & 0xff))
        _frame.data[4] = (_kp & 0xff)
        _frame.data[5] = (_kd >> 4)
        _frame.data[6] = (((_kd & 0x000f)<<4) + (_ff >> 8))
        _frame.data[7] = (_ff & 0xff)
        #print(_frame.data)
        return _frame

    def enable(self, id):
        _frame = frame(id)
        _frame.data[7]=ctypes.c_uint8(int(dgm.MOTOR_ENTER, 16)).value
        return _frame

    def disable(self, id):
        _frame = frame(id)
        _frame.data[7]=ctypes.c_uint8(int(dgm.MOTOR_EXIT, 16)).value
        return _frame

    def set_zero(self, id):
        _frame = frame(id)
        _frame.data[7]=ctypes.c_uint8(int(dgm.SET_ZERO, 16)).value
        return _frame

class decode():
    def __init__(self):
        pass

    def get_data(self, _frame):
        angle = 0.0
        speed = 0.0
        current = 0.0
        angle = ctypes.c_long((struct.unpack("B", _frame.data[1])[0]<<8) | struct.unpack("B", _frame.data[2])[0] ).value
        speed = ctypes.c_long((struct.unpack("B", _frame.data[3])[0]<<4) | ((struct.unpack("B", _frame.data[2])[0] & 0xf0) >> 4)).value
        current = ctypes.c_long(((struct.unpack("B", _frame.data[4])[0] & 0x0f) << 8) | struct.unpack("B", _frame.data[5])[0]).value
        _angle = _frame.map(angle, dgm.POSITION_CONTROL_HIGH_LIMIT, dgm.POSITION_CONTROL_LOW_LIMIT, dgm.POSITION_CONTROL_MIN, dgm.POSITION_CONTROL_MAX)
        _speed = _frame.map(speed, 0, dgm.BITS_12, dgm.SPEED_CONTROL_MIN, dgm.SPEED_CONTROL_MAX)
        _current = _frame.map(current, 0, dgm.BITS_12, dgm.CURRENT_MIN, dgm.CURRENT_MAX)
        return _angle, _speed, _current