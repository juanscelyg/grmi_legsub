#! /usr/bin/env python

import numpy as np
import rospy
from can_msgs.msg import Frame
from geometry_msgs.msg import WrenchStamped, TwistStamped, Vector3Stamped 
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from motor.can_motor import motor

class LegsubCANParse():
    def __init__(self):
        #Globals
        self.legs=1
        self.motors=4
        self.ids=[1,2,4,5]
        for i in range(len(self.ids)):
            motor(self.ids[i])


if __name__ == '__main__':
    rospy.init_node('legsub_can_parse')
    try:
        node = LegsubCANParse()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')