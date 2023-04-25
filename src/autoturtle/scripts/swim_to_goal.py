#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import numpy as np
import click
import math

class SubscriberNode(object):
    def __init__(self):
        self.data = None
        rospy.Subscriber('/turtle1/pose', Pose, self.callback)
        rospy.init_node('turtle_controller')
        # rospy.spin()

    def callback(self, data):
        self.data = data

def main_swim():
    rospy.loginfo('Starting Turtle Controller Node')
    # initialize needed nodes
    pos_node = SubscriberNode() # receives position feedback
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10) 
    # rospy.init_node('swim_controller') #publishes velocity commands
    r = rospy.Rate(40)
    move_cmd = Twist()

    # Controller gain
    Kp_x = 1.5
    Kp_less = 1
    Kp_lesser = 0.5
    Kp_z = 4

    while not rospy.is_shutdown():
        target_point = request_point()
        is_arrived = check_if_arrived(pos_node.data,target_point)
        iterations = 0
        if not is_arrived:
            is_facing_target = check_body_angle(pos_node.data,target_point)
            while not is_facing_target and not rospy.is_shutdown():
                # turn to face target
                is_facing_target = check_body_angle(pos_node.data,target_point)
                err_pos,err_ang = calc_error(pos_node.data,target_point)
                lin_vel = Kp_x*err_pos
                ang_vel = Kp_z*err_ang
                move_cmd.linear.x = 0
                move_cmd.angular.z = ang_vel

                if iterations % 10 == 0:
                    rospy.loginfo('err_pos = %.2f; err_ang = %.2f' % (err_pos,err_ang))
                    rospy.loginfo('lin_vel = %.2f; ang_vel = %.2f' % (lin_vel,ang_vel))
    
                pub.publish(move_cmd)
                iterations += 1
                r.sleep()

            while not is_arrived and not rospy.is_shutdown():
                # move to target
                is_arrived = check_if_arrived(pos_node.data,target_point)
                err_pos,err_ang = calc_error(pos_node.data,target_point)
                if abs(err_pos) < 1:
                    lin_vel = Kp_less*err_pos
                elif abs(err_pos) < 0.6:
                    lin_vel = Kp_lesser*err_pos
                else:
                    lin_vel = Kp_x*err_pos
                ang_vel = Kp_z*err_ang
                move_cmd.linear.x = lin_vel
                move_cmd.angular.z = 0

                if iterations % 5 == 0:
                    rospy.loginfo('err_pos = %.2f; err_ang = %.2f;' % (err_pos,err_ang))
                    # rospy.loginfo('lin_vel = %.2f; ang_vel = %.2f' % (lin_vel,ang_vel))
    
                pub.publish(move_cmd)
                iterations += 1
                r.sleep()

def calc_error(pose,target):
    x_err = target[0] - pose.x 
    y_err = target[1] - pose.y
    err_pos = math.sqrt(x_err**2 + y_err**2)
    # check sign of error_pos
    turtle_pos = np.array([pose.x,pose.y])
    target_vec = np.array([target[0],target[1]])
    vec_sub = target_vec - turtle_pos
    turtle_vec = np.array([pose.x*(math.cos(pose.theta)),pose.y*(math.sin(pose.theta))])
    dot_product = np.dot(turtle_vec,vec_sub)
    # rospy.loginfo('dot product = %.2f' % dot_product)
    if dot_product < 0:
        err_pos = -err_pos
    
    err_ang = math.atan2(y_err,x_err) - pose.theta 
    return err_pos, err_ang

def check_if_arrived(pose,target):
    err_pos,_ = calc_error(pose,target)
    if abs(err_pos) < 0.5:
        is_arrived = True
        rospy.loginfo('Arrived at target location: %.2f, %.2f with error %.2f' % (target[0],target[1],err_pos))
    else:
        is_arrived = False
    return is_arrived

def check_body_angle(pose,target):
    _,err_ang = calc_error(pose,target)
    if abs(err_ang) < 0.02:
        is_correct_angle = True
        rospy.loginfo('Facing target with orientation %.2f' % pose.theta) 
    else:
        is_correct_angle = False
    return is_correct_angle

def request_point():
    x_val = request_number('x')
    rospy.loginfo('entered %.2f' % x_val)
    y_val = request_number('y')
    rospy.loginfo('entered %.2f' % y_val)
    return (x_val, y_val)

def request_number(var_name):
    max_in = 9
    min_in = 1
    is_valid = False
    while not is_valid:
        val = click.prompt('Enter Target %s coordinate between 1-9 or q to exit' % var_name)
        try:
            val = float(val)
        except:
            if val == 'q':
                rospy.signal_shutdown('Node shutting down')
                out_val = 0
                break
            else:
                rospy.loginfo('Enter valid number characters')
            continue
        if val >= min_in and val <= max_in:
            out_val = val
            is_valid = True
        else:
            rospy.loginfo('Enter number between 0-9')

    return out_val


if __name__ == '__main__':
    main_swim()
