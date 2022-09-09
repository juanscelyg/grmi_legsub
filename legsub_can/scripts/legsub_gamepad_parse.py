#!/usr/bin/env python

import numpy as np
import rospy
import rosservice

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Vector3Stamped 
from std_srvs.srv import SetBool


class LegsubGamepadParseNode():
    def __init__(self):
        # ROS Infraestructure
        self.mytime = 0.1

        # Init constants
        self.flag_angles = False
        self.flag_move = False
        self.leg1_angle = 0.0
        self.leg2_angle = 0.0
        self.leg3_angle = 0.0
        self.leg1_vel = 0.0
        self.leg2_vel = 0.0

        # MODEL
        self.l = 0.223
        self.r = 0.0975
        self.jac_inv = np.matrix([[1/self.r, self.l/(self.r)],
        [1/self.r, -self.l/(self.r)]]);

        # ROS infraestucture
        self.sub_joy = rospy.Subscriber('/joy', Joy, self.get_values)
        self.pub_leg1 = rospy.Publisher('/can/0/leg/0/cmd_pos', Vector3Stamped, queue_size=5)
        self.pub_leg2 = rospy.Publisher('/can/0/leg/1/cmd_pos', Vector3Stamped, queue_size=5)
        self.pub_leg3 = rospy.Publisher('/can/0/leg/2/cmd_pos', Vector3Stamped, queue_size=5)
        self.pub_vel_leg1 = rospy.Publisher('/can/0/leg/0/cmd_vel', Vector3Stamped, queue_size=5)
        self.pub_vel_leg2 = rospy.Publisher('/can/0/leg/1/cmd_vel', Vector3Stamped, queue_size=5)

    def get_values(self, msg_joy):
        ref_hover = msg_joy.axes[4]
        ref_roll = msg_joy.axes[3]
        ref_yaw = msg_joy.axes[0]
        ref_moving = msg_joy.axes[1]
        # calc vel
        vel = np.matrix([[ref_moving],[ref_yaw]])
        [self.leg1_vel, self.leg2_vel] = np.matmul(self.jac_inv, vel)
        # calc ang
        self.leg1_angle = ref_hover - ref_roll
        self.leg2_angle = ref_hover + ref_roll
        self.leg3_angle = ref_hover + ref_roll

        #Special buttons
        if msg_joy.buttons[9] == 1:
            rosservice.call_service('/can/0/leg/0/set_init', SetBool, True)
            rosservice.call_service('/can/0/leg/1/set_init', SetBool, True)
            #rosservice.call_service('/can/0/leg/2/set_init', SetBool, True)
        if msg_joy.buttons[8] == 1:
            rosservice.call_service('/can/0/leg/0/zero', SetBool, True)
            rosservice.call_service('/can/0/leg/1/zero', SetBool, True)
            #rosservice.call_service('/can/0/leg/2/zero', SetBool, True)
        if msg_joy.buttons[7] == 1:
            self.flag_angles = True # Unlocked
            rospy.logwarn("Angle movements activated")
        else:
            self.flag_angles = False

        if msg_joy.buttons[6] == 1:
            self.flag_move = True # Unlocked
            rospy.logwarn("Moving movements activated")
        else:
            self.flag_move = False


    def pub_values(self, event):
        if self.flag_angles == True:
            msg_leg1 = Vector3Stamped()
            msg_leg1.header.stamp = rospy.Time.now()
            msg_leg1.vector.z = self.leg1_angle
            self.pub_leg1.publish(msg_leg1)
            msg_leg2 = Vector3Stamped()
            msg_leg2.header.stamp = rospy.Time.now()
            msg_leg2.vector.z = self.leg2_angle
            self.pub_leg2.publish(msg_leg2)
            '''
            msg_leg3 = Vector3Stamped()
            msg_leg3.header.stamp = rospy.Time.now()
            msg_leg4.vector.z = self.leg3_angle
            self.pub_leg3.publish(msg_leg3)            
            '''

        if self.flag_move == True:
            # vel 1 leg motor 3
            msg_leg1_vel = Vector3Stamped()
            msg_leg1_vel.header.stamp = rospy.Time.now()
            msg_leg1_vel.vector.z = self.leg1_vel
            self.pub_vel_leg1.publish(msg_leg1_vel)
            # vel 2 leg motor 6
            msg_leg2_vel = Vector3Stamped()
            msg_leg2_vel.header.stamp = rospy.Time.now()
            msg_leg2_vel.vector.z = -self.leg2_vel
            self.pub_vel_leg2.publish(msg_leg2_vel)

if __name__ == '__main__':
    rospy.init_node('legsub_gamepad_parse_node')
    try:
        node = LegsubGamepadParseNode()
        rospy.Timer(rospy.Duration(node.mytime), node.pub_values)
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')
