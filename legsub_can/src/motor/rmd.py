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
        self.acc = 0.0
        self.torque = 0.0
        self.temperature = 0.0
        self.position_kp = 0.0
        self.position_ki = 0.0
        self.speed_kp = 0.0
        self.speed_ki = 0.0
        self.torque_kp = 0.0
        self.torque_ki = 0.0
        self.encoder_position = 0.0
        self.encoder_original = 0.0
        self.encoder_offset = 0.0
        self.error = "NO ERRORS"
        self.error_state(0)
        self.voltage = 0.0
        self.mode=" "

    def error_state(self, index):
        if index == 1:
            self.error = "LOW VOLTAGE"
        elif index == 8:
            self.error = "OVER TEMPERATURE"
        else:
            self.error = "NO ERRORS"

class rmd():
    ENABLE = True
    DISABLE = False
    REDUCER_RATIO = 6.0
    #
    GET_ANGLE = "0x94"
    GET_MULTI_ANGLE = "0x92"
    GET_ACC = "0x33"
    GET_ENCODER = "0x90"
    GET_STATUS1 = "0x9A"
    GET_STATUS2 = "0x9C"
    TORQUE_CONTROL = "0xA1"
    SPEED_CONTROL = "0xA2"
    #
    POSITION_CONTROL = "0xA4"
    POSITION_CONTROL_HIGH_LIMIT = 36000
    POSITION_CONTROL_LOW_LIMIT = 0
    POSITION_CONTROL_MAX_SPEED = 100
    #
    POSITION_CONTROL_DIR = "0xA6"
    POSITION_CONTROL_DIR_HIGH_LIMIT = 36000
    POSITION_CONTROL_DIR_LOW_LIMIT = 0
    POSITION_CONTROL_DIR_MAX_SPEED = 100
    #
    SET_ZERO = "0x19"
    SET_ENCODER_OFFSET = "0x91" 
    MOTOR_RUNNING = "0x88"
    MOTOR_STOP = "0x81"
    MOTOR_OFF = "0x80"
    CLEAR_ERROR = "0x9B"
    GET_PID = "0x30"
    labels ={
        "0x94": "GET_ANGLE",
        "0x92": "GET_MULTI_ANGLE",
        "0x33": "GET_ACC",
        "0x90": "GET_ENCODER",
        "0x9a": "GET_STATUS1",
        "0x9c": "GET_STATUS2",
        "0xa1": "TORQUE_CONTROL",
        "0xa2": "SPEED_CONTROL",
        "0xa4": "POSITION_CONTROL",
        "0xa6": "POSITION_CONTROL_DIR",
        "0x19": "SET_ZERO",
        "0x91": "SET_ENCODER_OFFSET", 
        "0x88": "MOTOR_RUNNING",
        "0x81": "MOTOR_STOP",
        "0x80": "MOTOR_OFF",
        "0x9b": "CLEAR_ERROR",
        "0x30": "GET_PID"
    }

class frame():
    def __init__(self, id):
        self.id=ctypes.c_uint8(id).value
        self.real_id = ctypes.c_uint32(self.id + int("0x140", 16)).value
        self.is_extended = rmd.DISABLE
        self.dlc = ctypes.c_uint8(8).value
        self.data = []
        self.flush()

    def map(self,  x,  in_min,  in_max,  out_min,  out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    
    def flush(self):
        for i in range(8):
            self.data.append(ctypes.c_uint8(0).value)

class encode():
    def __init__(self):
        pass

    def get_angle(self,id):
        _frame = frame(id)
        _frame.data[0] = ctypes.c_uint8(int(rmd.GET_ANGLE, 16)).value
        return _frame

    def get_multi_angle(self, id):
        _frame = frame(id)
        _frame.data[0] = ctypes.c_uint8(int(rmd.GET_MULTI_ANGLE, 16)).value
        return _frame

    def set_position(self, id, d_angle):
        _frame = frame(id)
        '''
        angle_saturation = 20
        if abs(d_angle)>angle_saturation:
            d_angle = (d_angle/abs(d_angle))*angle_saturation
        '''
        d_motor_angle = _frame.map(d_angle, 90.0, -90.0, -540.0, 540.0)
        _frame.data[0] = ctypes.c_uint8(int(rmd.POSITION_CONTROL, 16)).value
        _frame.data[2] = rmd.POSITION_CONTROL_MAX_SPEED
        _frame.data[3] = (rmd.POSITION_CONTROL_MAX_SPEED>>8) & 0xff
        mapped_value = int(_frame.map(d_motor_angle, 0.0, 360.0, rmd.POSITION_CONTROL_LOW_LIMIT, rmd.POSITION_CONTROL_HIGH_LIMIT))
        _frame.data[4]= mapped_value & 0xff
        _frame.data[5]= (mapped_value>>8) & 0xff
        _frame.data[6]= (mapped_value>>16) & 0xff
        _frame.data[7]= (mapped_value>>24) & 0xff
        return _frame

    def set_position_dir(self, id, d_angle, angle):
        _frame = frame(id)
        '''
        angle_saturation = 20
        if abs(d_angle)>angle_saturation:
            d_angle = (d_angle/abs(d_angle))*angle_saturation
        d_motor_angle = _frame.map(d_angle, 20.0, -20.0, 60.0, 300.0)
        '''
        d_motor_angle = _frame.map(d_angle, 90.0, -90.0, -540.0, 540.0)
        if angle>180.0:
            _frame.data[1] = ctypes.c_uint8(1).value
        else:
            _frame.data[1] = ctypes.c_uint8(0).value
        _frame.data[0] = ctypes.c_uint8(int(rmd.POSITION_CONTROL_DIR, 16)).value
        _frame.data[2] = rmd.POSITION_CONTROL_DIR_MAX_SPEED
        _frame.data[3] = (rmd.POSITION_CONTROL_DIR_MAX_SPEED>>8) & 0xff
        mapped_value = int(_frame.map(d_motor_angle, 0.0, 360.0, rmd.POSITION_CONTROL_DIR_LOW_LIMIT, rmd.POSITION_CONTROL_DIR_HIGH_LIMIT))
        _frame.data[4]= mapped_value & 0xff
        _frame.data[5]= (mapped_value>>8) & 0xff
        return _frame

    def get_status1(self, id):
        _frame = frame(id)
        _frame.data[0]=ctypes.c_uint8(int(rmd.GET_STATUS1, 16)).value
        return _frame

    def get_status2(self, id):
        _frame = frame(id)
        _frame.data[0]=ctypes.c_uint8(int(rmd.GET_STATUS2, 16)).value
        return _frame

    def get_pid(self, id):
        _frame = frame(id)
        _frame.data[0]=ctypes.c_uint8(int(rmd.GET_PID, 16)).value
        return _frame

    def get_encoder(self, id):
        _frame = frame(id)
        _frame.data[0]=ctypes.c_uint8(int(rmd.GET_ENCODER, 16)).value
        return _frame

    def set_zero(self, id):
        _frame = frame(id)
        _frame.data[0]=ctypes.c_uint8(int(rmd.SET_ZERO, 16)).value
        return _frame

    def stop(self, id):
        _frame = frame(id)
        _frame.data[0]=ctypes.c_uint8(int(rmd.MOTOR_STOP, 16)).value
        return _frame
        

class decode():
    def __init__(self):
        pass

    def get_data(self, _motor, _frame):
        if struct.unpack("B", _frame.data[0])[0] == int(rmd.POSITION_CONTROL,16):
            _motor = self.set_angle(_motor, _frame)
        if struct.unpack("B", _frame.data[0])[0] == int(rmd.POSITION_CONTROL_DIR,16):
            _motor = self.set_angle(_motor, _frame)
        if struct.unpack("B", _frame.data[0])[0] == int(rmd.GET_ANGLE,16):
            _motor = self.get_angle(_motor, _frame)
        if struct.unpack("B", _frame.data[0])[0] == int(rmd.GET_MULTI_ANGLE,16):
            _motor = self.get_multi_angle(_motor, _frame)
        if struct.unpack("B", _frame.data[0])[0] == int(rmd.GET_STATUS1,16):
            _motor = self.get_status1(_motor, _frame)
        if struct.unpack("B", _frame.data[0])[0] == int(rmd.GET_STATUS2,16):
            _motor = self.set_angle(_motor, _frame)
        if struct.unpack("B", _frame.data[0])[0] == int(rmd.GET_PID,16):
            _motor = self.get_pid(_motor,_frame)
        if struct.unpack("B", _frame.data[0])[0] == int(rmd.GET_ENCODER,16):
            _motor = self.get_encoder(_motor,_frame)
        if struct.unpack("B", _frame.data[0])[0] == int(rmd.SET_ZERO,16):
            _motor = self.set_zero(_motor,_frame)
        if struct.unpack("B", _frame.data[0])[0] == int(rmd.MOTOR_STOP,16):
            _motor = self.stop(_motor,_frame)

        _motor.mode = rmd.labels[hex(struct.unpack("B", _frame.data[0])[0])]
        #print("Motor "+str(_frame.id)+" status := "+rmd.labels[hex(struct.unpack("B", _frame.data[0])[0])])
        return _motor

    def get_angle(self, _motor, _frame):
        angle = ctypes.c_long((struct.unpack("B", _frame.data[7])[0]<<8) | struct.unpack("B", _frame.data[6])[0] ).value
        _motor.angle = _frame.map(angle*0.01, 60.0, 300.0, 20.0, -20.0)
        return _motor

    def get_multi_angle(self, _motor, _frame):
        angle = ctypes.c_long((struct.unpack("B", _frame.data[7])[0]<<48) \
            | (struct.unpack("B", _frame.data[6])[0]<<40) | (struct.unpack("B", _frame.data[5])[0]<<32) \
                | (struct.unpack("B", _frame.data[4])[0]<<24) | (struct.unpack("B", _frame.data[3])[0]<<16) \
                    | (struct.unpack("B", _frame.data[2])[0]<<8) | (struct.unpack("B", _frame.data[1])[0]) ).value
        _motor.angle =  _frame.map(angle*0.01, 60.0, 300.0, 20.0, -20.0)
        return _motor

    def get_status1(self, _motor, _frame):
        _motor.temperature = struct.unpack("B", _frame.data[1])[0]
        voltage = ((struct.unpack("B", _frame.data[4])[0]<<8)  | struct.unpack("B", _frame.data[3])[0] )
        _motor.voltage = voltage*0.1
        _motor.error_state(struct.unpack("B", _frame.data[7])[0])
        return _motor

    def set_angle(self, _motor,_frame):
        _motor.temperature = struct.unpack("B", _frame.data[1])[0]
        torque = ctypes.c_long((struct.unpack("B", _frame.data[3])[0]<<8) | struct.unpack("B", _frame.data[2])[0] ).value
        _motor.torque = _frame.map(torque, -2048, 2048, -33.0, 33.0)
        speed = ctypes.c_long((struct.unpack("B", _frame.data[5])[0]<<8)  | struct.unpack("B", _frame.data[4])[0] ).value
        _motor.speed = speed #_frame.map(speed, -2048, 2048, -33, 33)
        angle = ctypes.c_long((struct.unpack("B", _frame.data[7])[0]<<8)| struct.unpack("B", _frame.data[6])[0] ).value
        _angle = _frame.map(angle, 0.0, 65535.0, 0.0, 360.0)
        _motor.angle = _frame.map(_angle, -540.0, 540.0, 90.0, -90.0, )
        return _motor

    def get_pid(self, _motor, _frame):
        _motor.position_kp = struct.unpack("B", _frame.data[2])[0]
        _motor.position_ki = struct.unpack("B", _frame.data[3])[0]
        _motor.speed_kp = struct.unpack("B", _frame.data[4])[0]
        _motor.speed_ki = struct.unpack("B", _frame.data[5])[0]
        _motor.torque_kp = struct.unpack("B", _frame.data[6])[0]
        _motor.torque_ki = struct.unpack("B", _frame.data[7])[0]
        return _motor

    def get_encoder(self, _motor, _frame):
        _motor.encoder_position = ((struct.unpack("B", _frame.data[3])[0]<<8) | struct.unpack("B", _frame.data[2])[0] )
        _motor.encoder_original = ((struct.unpack("B", _frame.data[5])[0]<<8)  | struct.unpack("B", _frame.data[4])[0] )
        _motor.encoder_offset = ((struct.unpack("B", _frame.data[7])[0]<<8)| struct.unpack("B", _frame.data[6])[0] )
        return _motor

    def set_zero(self, _motor,_frame):
        _motor.encoder_offset = ((struct.unpack("B", _frame.data[7])[0]<<8)| struct.unpack("B", _frame.data[6])[0] )
        return _motor

    def stop(self, _motor,_frame):
        pass
        return _motor
