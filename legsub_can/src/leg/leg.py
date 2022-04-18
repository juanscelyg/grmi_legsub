import numpy as np
import time

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3Stamped 

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
        for i in range(len(self.motors)):
            self.motors[i].get_state()

        self.sub_leg = rospy.Subscriber('/leg/'+str(self.ID)+'/init', Bool, self.call_leg)

    def begin(self):
        angle_set = 0.0
        if self.ids[2]==3:
            _angle = -angle_set
        if self.ids[2]==6:
            _angle = angle_set
        msg_pos = Vector3Stamped()
        msg_pos.vector.z = _angle
        self.motors[2].call_pos(msg_pos)


    def call_leg(self, msg):
        angle_set = 55.0
        if self.ids[2]==3:
            _angle = -angle_set
        if self.ids[2]==6:
            _angle = angle_set
        msg_pos = Vector3Stamped()
        msg_pos.vector.z = _angle
        self.motors[2].call_pos(msg_pos)    
    
        
