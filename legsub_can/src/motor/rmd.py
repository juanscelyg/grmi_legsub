#!/usr/bin/env python

import numpy as np
import math

class rmd():
    GET_ANGLE = "0x94"
    GET_MULTI_ANGLE = "0x92"
    GET_ACC = "0x33"
    GET_ENCODER = "0x90"
    GET_STATUS1 = "0x9A"
    GET_STATUS2 = "0x9C"
    TORQUE_CONTROL = "0xA1"
    SPEED_CONTROL = "0xA2"
    POSITION_CONTROL3 = "0xA5"
    SET_ZERO = "0x19"
    SET_ENCODER_OFFSET = "0x91" 
    MOTOR_RUNING = "0x88"
    MOTOR_STOP = "0x81"
    MOTOR_OFF = "0x80"
    CLEAR_ERROR = "0x9B"
    labels ={
        GET_ANGLE : "0x94",
        GET_MULTI_ANGLE : "0x92",
        GET_ACC : "0x33",
        GET_ENCODER : "0x90",
        GET_STATUS1 : "0x9A",
        GET_STATUS2 : "0x9C",
        TORQUE_CONTROL : "0xA1",
        SPEED_CONTROL : "0xA2",
        POSITION_CONTROL3 : "0xA5",
        SET_ZERO : "0x19",
        SET_ENCODER_OFFSET : "0x91", 
        MOTOR_RUNING : "0x88",
        MOTOR_STOP : "0x81",
        MOTOR_OFF : "0x80",
        CLEAR_ERROR : "0x9B"
    }

class frame():
    def __init__(self, id=1):
        self.id=np.uint8(id)
        self.real_id = np.uint32(self.id + int("0x140", 16))
        self.dlc = np.uint8(0)
        self.data = np.zeros((8,1), dtype=np.uint8)

class encode():
    def get_angle():
        return 0

    def get_status2():
        return 0
        

class decode():
    def get_angle():
        return 0

    def get_status2():
        return 0