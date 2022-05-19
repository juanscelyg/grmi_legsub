#!/usr/bin/env python

import time

import rospy
from std_srvs.srv import SetBool, SetBoolResponse
from geometry_msgs.msg import Vector3Stamped 
from can_msgs.msg import Frame
from motor.dgm import motor, dgm, frame, encode, decode

class joint():
    def __init__(self, can_network, leg_id, id):
        self.can_network = can_network
        self.leg_id = leg_id
        self.ID = id
        self.enabled = dgm.ENABLE
        self.mode = dgm.MOTOR_EXIT
        self.status = dgm.labels[dgm.MOTOR_EXIT]
        self.frame = frame(id)
        self.encode = encode()
        self.decode = decode()
        # motor parameters
        self.motor = motor()
        self.max_vel = 20.0
        self.kp = 500.0
        self.kd = 5.0
        self.ff = 10.0
        self.timer_time = 0.5

        ### ROS infrastructure 
        ## CAN
        self.sub_can = rospy.Subscriber('/can/'+str(self.can_network)+'/frame_out', Frame, self.call_can)
        self.pub_can = rospy.Publisher('/can/'+str(self.can_network)+'/frame_in', Frame, queue_size=5)
        ## ENVIRONMENT
        # MSGS
        # msgs in 
        self.sub_pos = rospy.Subscriber('/can/'+str(self.can_network)+'/leg/'+str(self.leg_id)+'/motor/'+str(self.ID)+'/cmd_pos', Vector3Stamped, self.call_pos)
        # msgs out
        self.pub_pos = rospy.Publisher('/can/'+str(self.can_network)+'/leg/'+str(self.leg_id)+'/motor/'+str(self.ID)+'/pos', Vector3Stamped, queue_size=5)
        # SERVICES 
        self.srv_state = rospy.Service('/can/'+str(self.can_network)+'/leg/'+str(self.leg_id)+'/motor/'+str(self.ID)+'/get_state', SetBool, self.call_state)
        self.srv_zero = rospy.Service('/can/'+str(self.can_network)+'/leg/'+str(self.leg_id)+'/motor/'+str(self.ID)+'/set_zero', SetBool, self.call_zero)
        self.srv_enable = rospy.Service('/can/'+str(self.can_network)+'/leg/'+str(self.leg_id)+'/motor/'+str(self.ID)+'/enable', SetBool, self.call_enable)
        ###
        
    ### ---------------------- PRINT FUNCTIONS ---------------------- ##

    def print_status(self):
        rospy.loginfo("----------------------------------------------------------")
        rospy.loginfo("Motor Network := %s" %self.can_network)
        rospy.loginfo("Motor ID := %s" %self.ID)
        rospy.loginfo("Motor pos := %s" %self.motor.angle)
        rospy.loginfo("Motor vel := %s" %self.motor.speed)
        rospy.loginfo("Motor current := %s" %self.motor.current)
        rospy.loginfo("Motor last mode := %s" %self.motor.mode)


    ### ---------------------- CAN FUNCTIONS ---------------------- ##

    def send2can(self, _frame):
        msg_frame = Frame()
        msg_frame.header.stamp = rospy.Time.now()
        msg_frame.id = _frame.real_id
        msg_frame.is_extended = _frame.is_extended
        msg_frame.dlc = _frame.dlc
        msg_frame.data = _frame.data
        self.pub_can.publish(msg_frame)
    
    def call_can(self, msg_can):
        if msg_can.id == self.frame.real_id:
            _frame = frame(msg_can.id)
            _frame.is_extended = msg_can.is_extended
            _frame.dlc = msg_can.dlc
            _frame.data = msg_can.data
            self.motor = self.decode.get_data(self.motor, _frame)

    ### ---------------------- CMD FUNCTIONS ---------------------- ##

    def call_pos(self, msg_pos):
        _frame = frame(self.ID)
        _angle = msg_pos.vector.z
        _frame = self.encode.set_angle(self.ID, id, _angle, self.max_vel, self.kp, self.kd, self.ff)
        self.motor.d_angle = _angle
        self.send2can(_frame)
        # crear msg de position

    def call_state(self, req):
        self.get_state()
        if req.data:
            self.print_status()
            return SetBoolResponse(req.data, 'QUERY:: can network: '+str(self.can_network)+' leg: '+str(self.leg_id)+' motor: '+str(self.ID)+' State was required. Results were displayed on screen.')
        else:
            return SetBoolResponse(req.data, 'QUERY:: can network: '+str(self.can_network)+' leg: '+str(self.leg_id)+' motor: '+str(self.ID)+' State was required. Results were not displayed on screen.')

    def call_zero(self, req):
        self.set_zero()
        if req.data:
            self.print_status()
            return SetBoolResponse(req.data, 'QUERY:: can network: '+str(self.can_network)+' leg: '+str(self.leg_id)+' motor: '+str(self.ID)+' Zero value was set. Results were displayed on screen.')
        else:
            return SetBoolResponse(req.data, 'QUERY:: can network: '+str(self.can_network)+' leg: '+str(self.leg_id)+' motor: '+str(self.ID)+' Zero value was set. Results were not displayed on screen.')

    def call_enable(self, req):
        self.set_zero()
        if req.data:
            self.enable()
            self.print_status()
            return SetBoolResponse(req.data, 'QUERY:: can network: '+str(self.can_network)+' leg: '+str(self.leg_id)+' motor: '+str(self.ID)+' was enabled. Now it is operative, the GREEN Led must be on.')
        else:
            self.disable()
            self.print_status()
            return SetBoolResponse(req.data, 'QUERY:: can network: '+str(self.can_network)+' leg: '+str(self.leg_id)+' motor: '+str(self.ID)+' was disabled. It is not operative, the RED Led must be on.')


    ### ---------------------- API FUNCTIONS ---------------------- ##

    def set_init(self):
        _frame = frame(self.ID)
        init_degree = 0.0
        _frame = self.encode.set_angle(self.ID, id, init_degree, self.max_vel, self.kp, self.kd, self.ff)
        self.motor.d_angle = init_degree
        self.send2can(_frame)

    def get_state(self):
        self.print_status()
        time.sleep(0.1)

    def set_zero(self):
        _frame = frame(self.ID)
        _frame = self.encode.set_zero(self.ID)
        self.send2can(_frame)

    def enable(self):
        _frame = frame(self.ID)
        _frame = self.encode.enable(self.ID)
        self.send2can(_frame)

    def disable(self):
        _frame = frame(self.ID)
        _frame = self.encode.disable(self.ID)
        self.send2can(_frame)