#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import random
import math

pi = 3.14

class SubscriberNode(object):
    def __init__(self):
        self.data = None
        rospy.Subscriber('/turtle1/pose', Pose, self.callback)
        rospy.init_node('figure_eight')

    def callback(self, data):
        self.prev_data = self.data
        self.data = data
        

def initialize_velocity():
    mean = 0.7
    stddev = 0.1
    lin_vel = random.normalvariate(mean, stddev)
    ang_vel = random.normalvariate(mean, stddev)
    rospy.loginfo('Lin Vel = %.2f, Ang Vel = %.2f' % (lin_vel,ang_vel))
    return lin_vel, ang_vel

def find_ang_iter(ang_vel,pub_rate):

    # we want to switch angular rotation direction every 2pi rotation of the turtle
    num_iter = 2*pi/ang_vel*pub_rate
    rospy.loginfo('num_iter = %.2f' % num_iter)
    num_iter = math.ceil(num_iter)
    rospy.loginfo('num_iter_rounded = %.2f' % num_iter)
    return num_iter    

def main_swim():
    rospy.loginfo('Starting swimmer')
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    # rospy.init_node('swim_commander')
    pos_node = SubscriberNode() # receives position feedback

    pub_rate = 20

    r = rospy.Rate(pub_rate)

    lin_vel, ang_vel = initialize_velocity()
    iter_rotate = find_ang_iter(ang_vel,pub_rate)
    iter_count = 0
    dir_shift = -1 #use this to alter the sign of angular velocity
    move_cmd = Twist()
    init_angle = pos_node.data.theta
    prev_angle = pos_node.data.theta
    ang_rotate_total = 0
    while not rospy.is_shutdown():
        # cur_angle = pos_node.data.theta
        # ang_rotate_iter = cur_angle - prev_angle
        # ang_rotate_total += ang_rotate_iter
        # rospy.loginfo('cur_angle = %.2f; prev_angle = %.2f; ang_rotate_iter = %.2f; ang_rotate_total = %.2f' % (cur_angle,prev_angle,ang_rotate_iter,ang_rotate_total))
        # prev_angle = cur_angle
        if iter_count > iter_rotate:
            # switch directions when needed
            ang_vel = ang_vel*dir_shift
            iter_count = 0
            rospy.loginfo('Switching directions')
        # if abs(ang_rotate_total) >= 2*pi:
        #     ang_vel = ang_vel*dir_shift
        #     ang_rotate_total = 0
        #     rospy.loginfo('Switching directions')
        
        move_cmd.linear.x = lin_vel
        move_cmd.angular.z = ang_vel
        pub.publish(move_cmd)
        r.sleep()
        iter_count += 1
if __name__ == '__main__':
    main_swim()


