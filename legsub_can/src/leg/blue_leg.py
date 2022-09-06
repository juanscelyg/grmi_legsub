import numpy as np
import time

import rospy
from geometry_msgs.msg import Vector3Stamped 
from std_srvs.srv import SetBool, SetBoolResponse

from motor.dgm_can_motor import joint

class config():
    can_net_id = 0
    A_type = np.array([1,2,3])
    B_type = np.array([4,5,6])
    C_type = np.array([7])

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

        # SERVICES
        self.srv_init = rospy.Service('/leg/'+str(self.ID)+'/init', SetBool, self.call_init)
        self.srv_begin = rospy.Service('/leg/'+str(self.ID)+'/begin', SetBool, self.call_begin)
        self.srv_zero = rospy.Service('/leg/'+str(self.ID)+'/zero', SetBool, self.call_zero)
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

    def call_zero(self, req):
        self.move_zero()
        if req.data:
            self.print_status()
            return SetBoolResponse(req.data, 'QUERY::  Leg: '+str(self.ID)+' Zero was required.')
        else:
            return SetBoolResponse(req.data, 'QUERY::  Leg: '+str(self.ID)+' Zero was required.')

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

    ### ---------------------- API FUNCTIONS ---------------------- ##

    def begin(self):
        for i in range(len(self.motors)):
            self.motors[i].enable()
        

    def set_init(self):
        if self.ID != 2:
            self.motors[0].set_init()
            self.motors[1].set_init()

    def move_zero(self):
        if self.ID != 2:
            self.motors[0].move_zero()
            self.motors[1].move_zero()

    def stop(self):
        for i in range(len(self.motors)):
            self.motors[i].stop()
    
        
