#!/usr/bin/env python

import numpy as np
import rospy
from can_msgs.msg import Frame
from geometry_msgs.msg import WrenchStamped, TwistStamped, Vector3Stamped 
from motor.rmd import rmd, frame, encode, decode

ENABLE = True
DISABLE = False

class motor():
    def __init__(self, id):
        self.ID = id
        self.enabled = ENABLE
        self.mode = rmd.GET_ANGLE
        self.status = rmd.labels(rmd.GET_ANGLE)
        self.angle = 0.0
        self.speed = 0.0
        self.acc = 0.0
        self.torque = 0.0
        self.frame = frame(id)
        rospy.loginfo("Motor id := %s" %self.frame.id)
        rospy.loginfo("Motor mode := %s" %self.mode)
        rospy.loginfo("Motor status := %s" %self.status)
        rospy.loginfo("Motor real_id := %s" %self.frame.real_id)

        # ROS infrastructure 
        # CAN
        self.sub_can = rospy.Subscriber('/frame_in', Frame, self.call_can)
        self.pub_can = rospy.Publisher('/frame_out', Frame, queue_size=1)
        # ENVIRONMENT
        self.sub_pos = rospy.Subscriber('/motor/'+str(self.ID)+'/cmd_pos', Vector3Stamped, self.call_pos)
        self.sub_vel = rospy.Subscriber('/motor/'+str(self.ID)+'/cmd_vel', TwistStamped, self.call_vel)
        self.sub_torque = rospy.Subscriber('/motor/'+str(self.ID)+'/cmd_torque', WrenchStamped, self.call_torque)
        self.pub_pos = rospy.Publisher('/motor/'+str(self.ID)+'/pos', Vector3Stamped, queue_size=1)
        self.pub_vel = rospy.Publisher('/motor/'+str(self.ID)+'/vel', TwistStamped, queue_size=1)
        self.pub_torque = rospy.Publisher('/motor/'+str(self.ID)+'/torque', WrenchStamped, queue_size=1)

    def call_can(self, msg_can):
        return 0

    def call_pos(self, msg_pos):
        return 0

    def call_vel(self, msg_vel):
        return 0

    def call_torque(self, msg_torque):
        return 0

