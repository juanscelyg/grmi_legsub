#!/usr/bin/env python

import numpy as np
import rospy
from can_msgs.msg import Frame
from geometry_msgs.msg import WrenchStamped, TwistStamped, Vector3Stamped 
from motor.rmd import motor, rmd, frame, encode, decode

class joint():
    def __init__(self, id):
        self.ID = id
        self.enabled = rmd.ENABLE
        self.mode = rmd.GET_ANGLE
        self.status = rmd.labels[rmd.GET_ANGLE]
        self.frame = frame(id)
        self.encode = encode()
        self.decode = decode()
        self.motor = motor()

        rospy.loginfo("Motor id := %s" %self.frame.id)
        rospy.loginfo("Motor mode := %s" %self.mode)
        rospy.loginfo("Motor status := %s" %self.status)
        rospy.loginfo("Motor real_id := %s" %self.frame.real_id)

        # ROS infrastructure 
        # CAN
        self.sub_can = rospy.Subscriber('/frame_out', Frame, self.call_can)
        self.pub_can = rospy.Publisher('/frame_in', Frame, queue_size=1)
        # ENVIRONMENT
        self.sub_pos = rospy.Subscriber('/motor/'+str(self.ID)+'/cmd_pos', Vector3Stamped, self.call_pos)
        self.sub_vel = rospy.Subscriber('/motor/'+str(self.ID)+'/cmd_vel', TwistStamped, self.call_vel)
        self.sub_torque = rospy.Subscriber('/motor/'+str(self.ID)+'/cmd_torque', WrenchStamped, self.call_torque)
        self.pub_pos = rospy.Publisher('/motor/'+str(self.ID)+'/pos', Vector3Stamped, queue_size=1)
        self.pub_vel = rospy.Publisher('/motor/'+str(self.ID)+'/vel', TwistStamped, queue_size=1)
        self.pub_torque = rospy.Publisher('/motor/'+str(self.ID)+'/torque', WrenchStamped, queue_size=1)


    def send2can(self, _frame):
        msg_frame=Frame()
        msg_frame.header.stamp = rospy.Time.now()
        msg_frame.id = _frame.real_id
        msg_frame.is_extended = _frame.is_extended
        msg_frame.dlc = _frame.dlc
        msg_frame.data = _frame.data
        #rospy.loginfo("Motor data := %s" %msg_frame.data)
        self.pub_can.publish(msg_frame)

    def call_can(self, msg_can):
        if msg_can.id == self.frame.real_id:
            _frame = frame(msg_can.id - int("0x140", 16))
            _frame.is_extended = msg_can.is_extended
            _frame.dlc = msg_can.dlc
            _frame.data = msg_can.data
            self.motor = self.decode.get_data(_frame)
            rospy.loginfo("Motor ID := %s" %self.ID)
            rospy.loginfo("Motor pos := %s" %self.motor.angle)
            rospy.loginfo("Motor vel := %s" %self.motor.speed)
            rospy.loginfo("Motor Torque := %s" %self.motor.torque)
            rospy.loginfo("Motor Temp := %s" %self.motor.temperature)


    def call_pos(self, msg_pos):
        _frame = frame(self.ID)
        _frame = self.encode.set_position(self.ID, msg_pos.vector.z)
        self.send2can(_frame)

    def call_vel(self, msg_vel):
        return 0

    def call_torque(self, msg_torque):
        return 0


