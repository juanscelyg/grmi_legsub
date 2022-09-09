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
        self.flag_gamepad = False
        self.leg1_angle = 0.0
        self.leg2_angle = 0.0
        self.leg3_angle = 0.0
        self.leg1_vel = 0.0
        self.leg2_vel = 0.0
        self.step = 0.05

        # MODEL
        self.l = 0.223
        self.r = 0.0975
        self.jac_inv = np.matrix([[1/self.r, self.l/(self.r)],
        [1/self.r, -self.l/(self.r)]]);

        # ROS infraestucture
        self.sub_joy = rospy.Subscriber('/joy', Joy, self.get_values)
        self.pub_leg1 = rospy.Publisher('/can/0/leg/0/cmd_pos', Vector3Stamped, queue_size=1)
        self.pub_leg2 = rospy.Publisher('/can/0/leg/1/cmd_pos', Vector3Stamped, queue_size=1)
        self.pub_leg3 = rospy.Publisher('/can/0/leg/2/cmd_pos', Vector3Stamped, queue_size=1)
        self.pub_vel_leg1 = rospy.Publisher('/can/0/leg/0/cmd_vel', Vector3Stamped, queue_size=1)
        self.pub_vel_leg2 = rospy.Publisher('/can/0/leg/1/cmd_vel', Vector3Stamped, queue_size=1)

        # ROS Service
        self.srv_leg1_init = rospy.ServiceProxy('/can/0/leg/0/init', SetBool)
        self.srv_leg2_init = rospy.ServiceProxy('/can/0/leg/1/init', SetBool)
        self.srv_leg3_init = rospy.ServiceProxy('/can/0/leg/2/init', SetBool)
        self.srv_leg1_zero = rospy.ServiceProxy('/can/0/leg/0/zero', SetBool)
        self.srv_leg2_zero = rospy.ServiceProxy('/can/0/leg/1/zero', SetBool)
        self.srv_leg3_zero = rospy.ServiceProxy('/can/0/leg/2/zero', SetBool)

    def get_values(self, msg_joy):
        self.ref_hover = self.ref_hover + msg_joy.button[2]*self.step - msg_joy.button[0]*self.step
        self.ref_roll = self.ref_roll + msg_joy.button[3]*self.step - msg_joy.button[1]*self.step
        self.ref_yaw = self.ref_yaw + msg_joy.button[14]*self.step - msg_joy.button[15]*self.step
        self.ref_moving = self.ref_moving + msg_joy.button[12]*self.step - msg_joy.button[13]*self.step
        # calc vel
        vel = np.matrix([[ref_moving],[ref_yaw]])
        [self.leg1_vel, self.leg2_vel] = np.matmul(self.jac_inv, vel)
        # calc ang
        self.leg1_angle = self.ref_hover - self.ref_roll
        self.leg2_angle = self.ref_hover + self.ref_roll
        self.leg3_angle = self.ref_hover 

        #Special buttons
        if msg_joy.buttons[4] == 1:
            self.flag_vel = True
        else:
            self.flag_vel = False
        if msg_joy.buttons[5] == 1:
            self.flag_angle = True
        else:
            self.flag_angle = False          
        if msg_joy.buttons[9] == 1:
            self.srv_leg1_init(True)
            self.srv_leg2_init(True)
            #self.srv_leg3_init(True)
        if msg_joy.buttons[8] == 1:
            self.srv_leg1_zero(True)
            self.srv_leg2_zero(True)
            #self.srv_leg3_zero(True)
        if msg_joy.buttons[6] == 1 and msg_joy.buttons[7] == 1:
            self.flag_gamepad = True # Unlocked
            rospy.logwarn("Gamepad has been activated")
        self.pub_values()


    def pub_values(self):
        if self.flag_gamepad == True:
            if self.flag_angle == True:
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
            if self.flag_vel == True:
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
        #rospy.Timer(rospy.Duration(node.mytime), node.pub_values)
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')
