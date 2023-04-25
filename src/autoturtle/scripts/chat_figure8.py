#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import Twist
import random
import math

pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
rospy.init_node('fig8')
rate = rospy.Rate(10) # 10 Hz
while not rospy.is_shutdown():
    # Move forward
    twist_msg = Twist()
    twist_msg.linear.x = 2.0
    pub.publish(twist_msg)
    rospy.sleep(1.0)
    
    # Turn left
    twist_msg = Twist()
    twist_msg.angular.z = 2.0
    pub.publish(twist_msg)
    rospy.sleep(1.0)
    
    # Move forward and turn right
    twist_msg = Twist()
    twist_msg.linear.x = 2.0
    twist_msg.angular.z = -2.0
    pub.publish(twist_msg)
    rospy.sleep(2.0)
    
    # Turn left
    twist_msg = Twist()
    twist_msg.angular.z = 2.0
    pub.publish(twist_msg)
    rospy.sleep(1.0)