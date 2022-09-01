#! /usr/bin/env python

import time
import rospy

from leg.blue_leg import config, leg

class BlueLegCANParse():
    def __init__(self):
        #Globals
        self.can_legs = [config.can_net_id, config.can_net_id, config.can_net_id]
        self.ids_legs = [config.A_type, config.B_type, config.C_type]
        self.blueleg_sub = []
        for i in range(len(self.can_legs)):
            self.blueleg_sub.append(leg(self.can_legs[i], i ,self.ids_legs[i]))
        for i in range(len(self.blueleg_sub)):
            self.blueleg_sub[i].begin()
        
if __name__ == '__main__':
    rospy.init_node('Blue_leg_can_parse')
    try:
        time.sleep(1)
        node = BlueLegCANParse()
        rate = rospy.Rate(10)
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')