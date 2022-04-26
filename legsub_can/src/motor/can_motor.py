#!/usr/bin/env python

import time

import rospy
from std_srvs.srv import SetBool, SetBoolResponse
from geometry_msgs.msg import Vector3Stamped 
from can_msgs.msg import Frame
from motor.rmd import motor, rmd, frame, encode, decode

class joint():
    def __init__(self, can_network, leg_id, id):
        self.can_network = can_network
        self.leg_id = leg_id
        self.ID = id
        self.enabled = rmd.ENABLE
        self.mode = rmd.GET_ANGLE
        self.status = rmd.labels[rmd.GET_ANGLE]
        self.frame = frame(id)
        self.encode = encode()
        self.decode = decode()
        self.motor = motor()
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
        ###
        
    ### ---------------------- PRINT FUNCTIONS ---------------------- ##

    def print_status(self):
        rospy.loginfo("----------------------------------------------------------")
        rospy.loginfo("Motor Network := %s" %self.can_network)
        rospy.loginfo("Motor ID := %s" %self.ID)
        rospy.loginfo("Motor pos := %s" %self.motor.angle)
        rospy.loginfo("Motor last mode := %s" %self.motor.mode)
        rospy.loginfo("Motor Temp := %s" %self.motor.temperature)
        rospy.loginfo("Motor Position Kp := %s" %self.motor.position_kp)
        rospy.loginfo("Motor Position Ki := %s" %self.motor.position_ki)
        rospy.loginfo("Motor encoder position := %s" %self.motor.encoder_position)
        rospy.loginfo("Motor encoder original := %s" %self.motor.encoder_original)
        rospy.loginfo("Motor encoder offset := %s" %self.motor.encoder_offset)
        rospy.loginfo("Motor error := %s" %self.motor.error)
        rospy.loginfo("Motor voltage := %s" %self.motor.voltage)

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
            _frame = frame(msg_can.id - int("0x140", 16))
            _frame.is_extended = msg_can.is_extended
            _frame.dlc = msg_can.dlc
            _frame.data = msg_can.data
            self.motor = self.decode.get_data(self.motor, _frame)

    ### ---------------------- CMD FUNCTIONS ---------------------- ##

    def call_pos(self, msg_pos):
        _frame = frame(self.ID)
        _angle = msg_pos.vector.z
        if self.ID == 3 or self.ID == 6:
            _angle = 2.0*_angle
        _frame = self.encode.set_position(self.ID, _angle)
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


    ### ---------------------- API FUNCTIONS ---------------------- ##

    def set_init(self):
        _frame = frame(self.ID)
        init_degree = 0.0
        _frame = self.encode.set_position(self.ID, init_degree)
        self.motor.d_angle = init_degree
        self.send2can(_frame)

    def get_state(self):
        self.get_multi()
        time.sleep(0.1)
        self.get_encoder()
        time.sleep(0.1)
        self.get_error()
        time.sleep(0.1)
        self.get_status()
        time.sleep(0.1)
        self.get_controller_status()
        time.sleep(0.1)

    def get_angle(self):
        _frame = frame(self.ID)
        _frame = self.encode.get_angle(self.ID)
        self.send2can(_frame)

    def get_multi(self):
        _frame = frame(self.ID)
        _frame = self.encode.get_multi_angle(self.ID)
        self.send2can(_frame)

    def get_controller_status(self):
        _frame = frame(self.ID)
        _frame = self.encode.get_pid(self.ID)
        self.send2can(_frame)

    def get_encoder(self):
        _frame = frame(self.ID)
        _frame = self.encode.get_encoder(self.ID)
        self.send2can(_frame)

    def get_status(self):
        _frame = frame(self.ID)
        _frame = self.encode.get_status2(self.ID)
        self.send2can(_frame)

    def get_error(self):
        _frame = frame(self.ID)
        _frame = self.encode.get_status1(self.ID)
        self.send2can(_frame)

    def set_zero(self):
        _frame = frame(self.ID)
        _frame = self.encode.set_zero(self.ID)
        self.send2can(_frame)

    def stop(self):
        _frame = frame(self.ID)
        _frame = self.encode.stop(self.ID)
        self.send2can(_frame)