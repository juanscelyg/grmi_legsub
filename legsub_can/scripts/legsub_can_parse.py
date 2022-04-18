#! /usr/bin/env python

import numpy as np
import time
import rospy
from can_msgs.msg import Frame
from geometry_msgs.msg import WrenchStamped, TwistStamped, Vector3Stamped 
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from motor.can_motor import joint
from leg.leg import config, leg

class LegsubCANParse():
    def __init__(self):
        #Globals
        self.can_legs = [config.can3, config.can3, config.can5, config.can5]
        self.ids_legs = [config.A_type, config.B_type, config.A_type, config.B_type]
        self.leg_sub = []
        for i in range(len(self.can_legs)):
            self.leg_sub.append(leg(self.can_legs[i], i ,self.ids_legs[i]))
        for i in range(len(self.leg_sub)):
            self.leg_sub[i].begin()
            #rospy.loginfo("Leg %s has began " %str(i+1))
        


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