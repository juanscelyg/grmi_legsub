#!/usr/bin/env python

import numpy as np
import ctypes
import struct
import math

class motor():
    def __init__(self):
        self.angle = 0.0
        self.speed = 0.0
        self.acc = 0.0
        self.torque = 0.0
        self.temperature = 0.0


class rmd():
    ENABLE = True
    DISABLE = False
    REDUCER_RATIO = 6.0
    GET_ANGLE = "0x94"
    GET_MULTI_ANGLE = "0x92"
    GET_ACC = "0x33"
    GET_ENCODER = "0x90"
    GET_STATUS1 = "0x9A"
    GET_STATUS2 = "0x9C"
    TORQUE_CONTROL = "0xA1"
    SPEED_CONTROL = "0xA2"
    #
    POSITION_CONTROL4 = "0xA6"
    POSITION_CONTROL4_HIGH_LIMIT = 35999
    POSITION_CONTROL4_LOW_LIMIT = 0
    POSITION_CONTROL4_SPEED_HIGH_LIMIT = 10
    POSITION_CONTROL4_SPEED_LOW_LIMIT = 0
    #
    SET_ZERO = "0x19"
    SET_ENCODER_OFFSET = "0x91" 
    MOTOR_RUNING = "0x88"
    MOTOR_STOP = "0x81"
    MOTOR_OFF = "0x80"
    CLEAR_ERROR = "0x9B"
    labels ={
        "0x94": "GET_ANGLE",
        "0x92": "GET_MULTI_ANGLE",
        "0x33": "GET_ACC",
        "0x90": "GET_ENCODER",
        "0x9A": "GET_STATUS1",
        "0x9C": "GET_STATUS2",
        "0xA1": "TORQUE_CONTROL",
        "0xA2": "SPEED_CONTROL",
        "0xA6": "POSITION_CONTROL4",
        "0x19": "SET_ZERO",
        "0x91": "SET_ENCODER_OFFSET", 
        "0x88": "MOTOR_RUNING",
        "0x81": "MOTOR_STOP",
        "0x80": "MOTOR_OFF",
        "0x9B": "CLEAR_ERROR"
    }

class frame():
    def __init__(self, id):
        self.id=ctypes.c_uint8(id).value
        self.real_id = ctypes.c_uint32(self.id + int("0x140", 16)).value
        self.is_extended = rmd.DISABLE
        self.dlc = ctypes.c_uint8(8).value
        self.data = []
        for i in range(8):
            self.data.append(ctypes.c_uint8(0).value)

    def map(self,  x,  in_min,  in_max,  out_min,  out_max):
        return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

class encode():
    def __init__(self):
        pass

    def get_angle(self,id):
        _frame = frame(id)
        _frame.data[0] = ctypes.c_uint8(int(rmd.GET_ANGLE, 16)).value
        return _frame

    def set_position(self, id, angle):
        _frame = frame(id)
        mapped_value = _frame.map(abs(angle), 0, 359.99, rmd.POSITION_CONTROL4_LOW_LIMIT, rmd.POSITION_CONTROL4_HIGH_LIMIT)
        _frame.data[0]=ctypes.c_uint8(int(rmd.POSITION_CONTROL4, 16)).value
        if angle>0:
            _frame.data[1]=ctypes.c_uint8(0).value
        else:
            _frame.data[1]=ctypes.c_uint8(1).value
        _frame.data[2]=rmd.POSITION_CONTROL4_SPEED_LOW_LIMIT & 0xff
        _frame.data[3]=(rmd.POSITION_CONTROL4_SPEED_HIGH_LIMIT>>8) & 0xff
        _frame.data[4]=mapped_value & 0xff
        _frame.data[5]=(mapped_value>>8) & 0xff
        return _frame


    def get_status2(self):
        return 0
        

class decode():
    def get_data(self, _frame):
        _motor = motor()
        if struct.unpack("B", _frame.data[0])[0] == int(rmd.POSITION_CONTROL4,16):
            _motor = self.set_angle(_motor, _frame)
        return _motor

    def set_angle(self, _motor,_frame):
        _motor.temperature = struct.unpack("B", _frame.data[1])[0]
        torque = ((struct.unpack("B", _frame.data[3])[0]<<8) | struct.unpack("B", _frame.data[2])[0] )
        _motor.torque = _frame.map(torque, -2048, 2048, -33, 33)
        speed = ((struct.unpack("B", _frame.data[5])[0]<<8)  | struct.unpack("B", _frame.data[4])[0] )
        _motor.speed = speed #_frame.map(speed, -2048, 2048, -33, 33)
        angle = ((struct.unpack("B", _frame.data[7])[0]<<8)| struct.unpack("B", _frame.data[6])[0] )
        _motor.angle = _frame.map(angle, 0, 16383, 0, 100)
        print(_motor.temperature)
        print(_motor.torque)
        print(_motor.speed)
        print(_motor.angle)
        return _motor

    def get_status2(self):
        return 0