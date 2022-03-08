#! /usr/bin/env python

import numpy as np
import rospy
from can_msgs.msg import Frame
from geometry_msgs.msg import WrenchStamped, TwistStamped, Vector3Stamped 
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from motor.can_motor import joint

class LegsubCANParse():
    def __init__(self):
        #Globals
        self.legs=1
        self.ids=[1,2,4,5]
        self.n_motors=len(self.ids)
        self.motor=[]
        for i in range(self.n_motors):
            self.motor.append(joint(self.ids[i]))


if __name__ == '__main__':
    rospy.init_node('legsub_can_parse')
    try:
        node = LegsubCANParse()
        rate = rospy.Rate(10)
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')