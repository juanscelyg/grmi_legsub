import numpy as np
import time

import rospy
from geometry_msgs.msg import Vector3Stamped 
from std_srvs.srv import SetBool, SetBoolResponse

from motor.can_motor import joint

class config():
    can3 = 0
    can5 = 1
    A_type = np.array([1,2,3])
    B_type = np.array([4,5,6])

class leg():
    def __init__(self, can_network=0, leg_id=0, ids_vector = np.array([1,2,3])):
        self.can_network = can_network
        self.ID = leg_id
        self.ids = ids_vector
        self.n_motors=len(self.ids)
        self.motors=[]
        for i in range(self.n_motors):
            self.motors.append(joint(self.can_network, self.ID, self.ids[i]))
        time.sleep(1)
        self.print_status()

        # SERVICES
        self.srv_init = rospy.Service('/leg/'+str(self.ID)+'/init', SetBool, self.call_init)
        self.srv_begin = rospy.Service('/leg/'+str(self.ID)+'/begin', SetBool, self.call_begin)
        self.srv_stop = rospy.Service('/leg/'+str(self.ID)+'/stop', SetBool, self.call_stop)

    ### ---------------------- CMD FUNCTIONS ---------------------- ##
    def call_init(self, req):
        self.set_init()
        if req.data:
            self.print_status()
            return SetBoolResponse(req.data, 'QUERY::  Leg: '+str(self.ID)+' Init was required.')
        else:
            return SetBoolResponse(req.data, 'QUERY::  Leg: '+str(self.ID)+' Init was required.')

    def call_begin(self, req):
        self.begin()
        if req.data:
            self.print_status()
            return SetBoolResponse(req.data, 'QUERY::  Leg: '+str(self.ID)+' Begin was required.')
        else:
            return SetBoolResponse(req.data, 'QUERY::  Leg: '+str(self.ID)+' Begin was required.')

    def call_stop(self, req):
        self.set_init()
        if req.data:
            self.print_status()
            return SetBoolResponse(req.data, 'QUERY::  Leg: '+str(self.ID)+' Init was required.')
        else:
            return SetBoolResponse(req.data, 'QUERY::  Leg: '+str(self.ID)+' Init was required.')

    ### ---------------------- PRINT FUNCTIONS ---------------------- ##

    def print_status(self):
        for i in range(len(self.motors)):
            self.motors[i].get_state()
            self.motors[i].print_status()

    ### ---------------------- API FUNCTIONS ---------------------- ##

    def begin(self):
        angle_set = 0.0
        if self.ids[2]==3:
            _angle = angle_set
        if self.ids[2]==6:
            _angle = -angle_set
        msg_pos = Vector3Stamped()
        msg_pos.vector.z = _angle
        self.motors[2].call_pos(msg_pos)
        _angle = 0.0
        msg_pos.vector.z = _angle
        self.motors[0].call_pos(msg_pos)
        msg_pos.vector.z = _angle*-1.0
        self.motors[1].call_pos(msg_pos)
        

    def set_init(self):
        angle_set = 55.0
        if self.ids[2]==3:
            _angle = angle_set
        if self.ids[2]==6:
            _angle = -angle_set
        msg_pos = Vector3Stamped()
        msg_pos.vector.z = _angle
        self.motors[2].call_pos(msg_pos)   
        _angle = 0.0
        msg_pos.vector.z = _angle
        self.motors[0].call_pos(msg_pos)
        msg_pos.vector.z = _angle*-1.0
        self.motors[1].call_pos(msg_pos) 

    def stop(self):
        for i in range(len(self.motors)):
            self.motors[i].stop()
    
        
