#! /usr/bin/env python

import numpy as np
import time
import rospy
from can_msgs.msg import Frame
from geometry_msgs.msg import WrenchStamped, TwistStamped, Vector3Stamped 
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from motor.can_motor import joint

class LegsubCANParse():
    def __init__(self):
        #Globals
        self.legs=1
        #self.ids=[1,2,3,4,5,6]
        self.ids=[3,6]
        self.n_motors=len(self.ids)
        self.motor=[]
        for i in range(self.n_motors):
            self.motor.append(joint(self.ids[i]))
        time.sleep(1)
        for i in range(len(self.motor)):
            self.motor[i].get_state()        


if __name__ == '__main__':
    rospy.init_node('legsub_can_parse')
    try:
        time.sleep(1)
        node = LegsubCANParse()
        rate = rospy.Rate(10)
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')