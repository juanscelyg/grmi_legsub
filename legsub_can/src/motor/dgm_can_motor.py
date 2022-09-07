#!/usr/bin/env python

import time
import struct

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
        self.motor.mode = dgm.labels[dgm.MOTOR_EXIT]
        self.motor.zero = dgm.ZERO
        self.motor.d_angle = dgm.ZERO
        self.max_vel = rospy.get_param("motor"+str(id)+"/max_vel")
        self.kp  = rospy.get_param("motor"+str(id)+"/kp")
        self.kd  = rospy.get_param("motor"+str(id)+"/kd")
        self.ff  = rospy.get_param("motor"+str(id)+"/ff")
        self.offset = rospy.get_param("motor"+str(id)+"/offset")
        self.timer_time = 0.5

        ### ROS infrastructure 
        ## CAN
        self.sub_can = rospy.Subscriber('/can/'+str(self.can_network)+'/frame_out', Frame, self.call_can)
        self.pub_can = rospy.Publisher('/can/'+str(self.can_network)+'/frame_in', Frame, queue_size=5)
        ## ENVIRONMENT
        # MSGS
        # msgs in 
        self.sub_pos = rospy.Subscriber('/can/'+str(self.can_network)+'/leg/'+str(self.leg_id)+'/motor/'+str(self.ID)+'/cmd_pos', Vector3Stamped, self.call_pos)
        if self.ID == 3 or self.ID == 6:
            self.sub_vel = rospy.Subscriber('/can/'+str(self.can_network)+'/leg/'+str(self.leg_id)+'/motor/'+str(self.ID)+'/cmd_vel', Vector3Stamped, self.call_vel)
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
        _id = struct.unpack("B", msg_can.data[0])[0]
        if _id == self.ID:
            _frame = frame(_id)
            _frame.is_extended = msg_can.is_extended
            _frame.dlc = msg_can.dlc
            _frame.data = msg_can.data
            angle, speed, current = self.decode.get_data(_frame)
            self.motor.angle = angle
            self.motor.speed = speed
            self.motor.current = current         

    ### ---------------------- CMD FUNCTIONS ---------------------- ##

    def call_pos(self, msg_pos):
        _frame = frame(self.ID)
        _angle = msg_pos.vector.z
        _frame = self.encode.set_angle(self.ID, _angle, self.max_vel, self.kp, self.kd, self.ff)
        self.motor.d_angle = _angle
        self.motor.mode = dgm.labels[dgm.POSITION_CONTROL]
        self.send2can(_frame)
        # visualize
        self.get_state()
        # Publish angle
        msg_angle = Vector3Stamped()
        msg_angle.header.stamp = rospy.Time.now()
        msg_angle.vector.z = self.motor.angle
        self.pub_pos.publish(msg_angle)

    def call_vel(self, msg_vel):
        _frame = frame(self.ID)
        _vel = msg_vel.vector.z
        _frame = self.encode.set_angle(self.ID, 0.0, _vel, self.kp, self.kd, self.ff)
        self.motor.d_vel = _vel
        self.motor.mode = dgm.labels[dgm.SPEED_CONTROL]
        self.send2can(_frame)
        # visualize
        self.get_state()


    def call_state(self, req):
        self.print_status()
        return SetBoolResponse(req.data, 'QUERY:: can network: '+str(self.can_network)+' leg: '+str(self.leg_id)+' motor: '+str(self.ID)+' State was required. Results were displayed on screen.')

    def call_zero(self, req):
        self.motor.mode = dgm.labels[dgm.SET_ZERO]
        self.set_zero()
        if req.data:
            self.print_status()
            return SetBoolResponse(req.data, 'QUERY:: can network: '+str(self.can_network)+' leg: '+str(self.leg_id)+' motor: '+str(self.ID)+' Zero value was set. Results were displayed on screen.')
        else:
            return SetBoolResponse(req.data, 'QUERY:: can network: '+str(self.can_network)+' leg: '+str(self.leg_id)+' motor: '+str(self.ID)+' Zero value was set. Results were not displayed on screen.')

    def call_enable(self, req):
        if req.data:
            self.motor.mode = dgm.labels[dgm.MOTOR_ENTER]
            self.enable()
            self.print_status()
            return SetBoolResponse(req.data, 'QUERY:: can network: '+str(self.can_network)+' leg: '+str(self.leg_id)+' motor: '+str(self.ID)+' was enabled. Now it is operative, the GREEN Led must be on.')
        else:
            self.motor.mode = dgm.labels[dgm.MOTOR_EXIT]
            self.disable()
            self.print_status()
            return SetBoolResponse(req.data, 'QUERY:: can network: '+str(self.can_network)+' leg: '+str(self.leg_id)+' motor: '+str(self.ID)+' was disabled. It is not operative, the RED Led must be on.')


    ### ---------------------- API FUNCTIONS ---------------------- ##

    def set_init(self):
        _frame = frame(self.ID)
        init_degree = self.motor.zero
        self.motor.d_angle = init_degree
        _frame = self.encode.set_angle(self.ID, init_degree, self.max_vel/10.0, self.kp, self.kd, self.ff)
        self.motor.mode = dgm.labels[dgm.MOTOR_ENTER]
        self.send2can(_frame)

    def move_zero(self):
        _frame = frame(self.ID)
        init_degree = self.offset
        self.motor.d_angle = init_degree
        _frame = self.encode.set_angle(self.ID, init_degree, self.max_vel/10.0, self.kp, self.kd, self.ff)
        self.motor.mode = dgm.labels[dgm.MOTOR_ENTER]
        self.send2can(_frame)

    def stop(self):
        _frame = frame(self.ID)
        _vel = 0.0
        _frame = self.encode.set_angle(self.ID, 0.0, _vel, self.kp, self.kd, self.ff)
        self.motor.d_vel = _vel
        self.motor.mode = dgm.labels[dgm.SPEED_CONTROL]
        self.send2can(_frame)

    def get_state(self):
        time.sleep(0.1)
        self.print_status()    

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