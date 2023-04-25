#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import Twist
import random
import math

def initialize_velocity():
    mean = 0.7
    stddev = 0.1
    lin_vel = random.normalvariate(mean, stddev)
    ang_vel = random.normalvariate(mean, stddev)
    rospy.loginfo('Lin Vel = %.2f, Ang Vel = %.2f' % (lin_vel,ang_vel))
    return lin_vel, ang_vel

def find_ang_iter(ang_vel,pub_rate):
    pi = 3.14
    # we want to switch angular rotation direction every 2pi rotation of the turtle
    num_iter = 2*pi/ang_vel*pub_rate
    rospy.loginfo('num_iter = %.2f' % num_iter)
    num_iter = math.ceil(num_iter)
    rospy.loginfo('num_iter_rounded = %.2f' % num_iter)
    return num_iter    

def main_swim():
    rospy.loginfo('Starting swimmer')
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.init_node('swim_commander')
    pub_rate = 20
    r = rospy.Rate(pub_rate)

    lin_vel, ang_vel = initialize_velocity()
    iter_rotate = find_ang_iter(ang_vel,pub_rate)
    iter_count = 0
    dir_shift = -1 #use this to alter the sign of angular velocity
    move_cmd = Twist()
    while not rospy.is_shutdown():
        
        if iter_count > iter_rotate:
            # switch directions when needed
            ang_vel = ang_vel*dir_shift
            iter_count = 0
            rospy.loginfo('Switching directions')
        
        move_cmd.linear.x = lin_vel
        move_cmd.angular.z = ang_vel
        pub.publish(move_cmd)
        r.sleep()
        iter_count += 1
if __name__ == '__main__':
    main_swim()


