#!/usr/bin/env python

import time

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import WrenchStamped, TwistStamped, Vector3Stamped 
from can_msgs.msg import Frame
from control_msgs.msg import PidState
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
        self.timer_time = 0.5

        ### ROS infrastructure 
        # CAN
        self.sub_can = rospy.Subscriber('/frame_out', Frame, self.call_can)
        self.pub_can = rospy.Publisher('/frame_in', Frame, queue_size=5)
        # ENVIRONMENT
        self.sub_pos = rospy.Subscriber('/motor/'+str(self.ID)+'/cmd_pos', Vector3Stamped, self.call_pos)
        self.sub_vel = rospy.Subscriber('/motor/'+str(self.ID)+'/cmd_vel', TwistStamped, self.call_vel)
        self.sub_torque = rospy.Subscriber('/motor/'+str(self.ID)+'/cmd_torque', WrenchStamped, self.call_torque)
        self.sub_init = rospy.Subscriber('/motor/'+str(self.ID)+'/init', Bool, self.call_init)
        ###
        self.sub_angle = rospy.Subscriber('/motor/'+str(self.ID)+'/get_angle', Bool, self.call_angle)
        self.sub_multi = rospy.Subscriber('/motor/'+str(self.ID)+'/get_multi_angle', Bool, self.call_multi)
        self.sub_state = rospy.Subscriber('/motor/'+str(self.ID)+'/get_state', Bool, self.call_state)
        ###
        self.sub_pid = rospy.Subscriber('/motor/'+str(self.ID)+'/controller/get_pid', Bool, self.call_controller_status)
        self.sub_encoder = rospy.Subscriber('/motor/'+str(self.ID)+'/controller/get_encoder', Bool, self.call_encoder)
        self.sub_status2 = rospy.Subscriber('/motor/'+str(self.ID)+'/controller/get_status', Bool, self.call_status)
        self.sub_error = rospy.Subscriber('/motor/'+str(self.ID)+'/controller/get_error', Bool, self.call_error)
        self.sub_zero = rospy.Subscriber('/motor/'+str(self.ID)+'/controller/set_zero', Bool, self.call_zero)
        ###
        self.pub_pos = rospy.Publisher('/motor/'+str(self.ID)+'/pos', Vector3Stamped, queue_size=1)
        self.pub_vel = rospy.Publisher('/motor/'+str(self.ID)+'/vel', TwistStamped, queue_size=1)
        self.pub_torque = rospy.Publisher('/motor/'+str(self.ID)+'/torque', WrenchStamped, queue_size=1)
        self.pub_control = rospy.Publisher('/motor/'+str(self.ID)+'/controller/status', PidState, queue_size=1)


    ### ---------------------- PRINT FUNCTIONS ---------------------- ##

    def print_status(self):
        rospy.loginfo("----------------------------------------------------------")
        rospy.loginfo("Motor ID := %s" %self.ID)
        rospy.loginfo("Motor pos := %s" %self.motor.angle)
        rospy.loginfo("Motor vel := %s" %self.motor.speed)
        rospy.loginfo("Motor Torque := %s" %self.motor.torque)
        rospy.loginfo("Motor Temp := %s" %self.motor.temperature)
        rospy.loginfo("Motor Position Kp := %s" %self.motor.position_kp)
        rospy.loginfo("Motor Position Ki := %s" %self.motor.position_ki)
        rospy.loginfo("Motor encoder position := %s" %self.motor.encoder_position)
        rospy.loginfo("Motor encoder original := %s" %self.motor.encoder_original)
        rospy.loginfo("Motor encoder offset := %s" %self.motor.encoder_offset)
        rospy.loginfo("Motor error := %s" %self.motor.error)
        rospy.loginfo("Motor voltage := %s" %self.motor.voltage)

    def get_state(self):
        msg=[]
        self.call_angle(msg)
        time.sleep(0.1)
        self.call_encoder(msg)
        time.sleep(0.1)
        self.call_error(msg)
        time.sleep(0.1)
        self.call_status(msg)
        time.sleep(0.1)
        self.print_status()

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
        _frame = self.encode.set_position(self.ID, msg_pos.vector.z)
        self.motor.d_angle = msg_pos.vector.z
        self.send2can(_frame)


    def call_vel(self, msg_vel):
        return 0

    def call_torque(self, msg_torque):
        return 0

    ### ---------------------- CALLBACK FUNCTIONS ---------------------- ##

    def call_init(self, msg):
        _frame = frame(self.ID)
        init_degree = 0.0
        _frame = self.encode.set_position_dir(self.ID, init_degree, self.motor.angle)
        self.motor.d_angle = init_degree
        self.send2can(_frame)

    def call_state(self, msg_state):
        self.get_state()

    def call_angle(self, msg_con):
        _frame = frame(self.ID)
        _frame = self.encode.get_angle(self.ID)
        self.send2can(_frame)

    def call_multi(self, msg_con):
        _frame = frame(self.ID)
        _frame = self.encode.get_multi_angle(self.ID)
        self.send2can(_frame)

    def call_controller_status(self, msg_con):
        _frame = frame(self.ID)
        _frame = self.encode.get_pid(self.ID)
        self.send2can(_frame)

    def call_encoder(self, msg_enc):
        _frame = frame(self.ID)
        _frame = self.encode.get_encoder(self.ID)
        self.send2can(_frame)

    def call_status(self, msg_status):
        _frame = frame(self.ID)
        _frame = self.encode.get_status2(self.ID)
        self.send2can(_frame)

    def call_error(self, msg_enc):
        _frame = frame(self.ID)
        _frame = self.encode.get_status1(self.ID)
        self.send2can(_frame)

    def call_zero(self, msg_enc):
        _frame = frame(self.ID)
        _frame = self.encode.set_zero(self.ID)
        self.send2can(_frame)

        


