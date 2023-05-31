#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import numpy as np
import click
import math
import PID_Controller as pid

class SubscriberNode(object):
    def __init__(self):
        self.data = None
        rospy.Subscriber('/turtle1/pose', Pose, self.callback)
        rospy.init_node('turtle_controller')

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

def calc_error(cur_state,target):
    # cur_state is a dictonary with field pose (geometry_msg Point, and field theta with a radian angle 
    # target is a length 2 iterable that has the x and y coordinates of final location
    pose = cur_state['pose']
    theta = cur_state['theta']
    x_err, y_err = calc_vec_err(cur_state, target)
    err_pos = math.sqrt(x_err**2 + y_err**2)
    # check sign of error_pos
    turtle_pos = np.array([pose.x,pose.y])
    target_vec = np.array([target[0],target[1]])
    # vec_sub = target_vec - turtle_pos # I think this one is the wrong sign
    vec_sub = turtle_pos - target_vec
    # turtle_vec = np.array([pose.x*(math.cos(theta)),pose.y*(math.sin(theta))])
    turtle_vec = np.array([math.cos(theta),math.sin(theta)])
    dot_product = np.dot(turtle_vec,vec_sub)
    # rospy.loginfo('dot product = %.2f' % dot_product)
    if dot_product > 0:
        err_pos = -err_pos
        if theta >= 0:
            theta -= math.pi
        else:
            theta += math.pi
    
    ref_angle = math.atan2(y_err,x_err)
    err_ang = calc_ang_error(theta, ref_angle)

    # pid.debug_info("calc_error",theta_ref=math.atan2(y_err,x_err),cur_theta=theta)
    return err_pos, err_ang

def calc_ang_error(theta, ref_angle):
    err_ang = ref_angle - theta 
    if err_ang > math.pi:
        err_ang = -1*(2*math.pi-err_ang)
    elif err_ang < -math.pi:
        err_ang = -1*(-2*math.pi-err_ang)
    return err_ang

def calc_vec_err(cur_state, target):
    x_err = target[0] - cur_state['pose'].x 
    y_err = target[1] - cur_state['pose'].y
    return x_err,y_err

def check_if_arrived(pose,target):
    err_pos,_ = calc_error(pose,target)
    if abs(err_pos) <= 0.05:
        is_arrived = True
        # rospy.loginfo('Arrived at target location: %.2f, %.2f with error %.3f' % (target[0],target[1],err_pos))
    else:
        is_arrived = False
    return is_arrived

def check_body_angle(pose,target):
    _,err_ang = calc_error(pose,target)
    if abs(err_ang) <= math.pi/180:
        is_correct_angle = True
        # rospy.loginfo('Facing target with orientation %.2f' % pose['theta']) 
    else:
        is_correct_angle = False
    return is_correct_angle

def request_point():
    x_val = request_number('x')
    y_val = request_number('y')

    return (x_val, y_val)

def request_number(var_name,is_bounds=True,bounds=(1,9)):
    max_in = bounds[1]
    min_in = bounds[0]
    is_valid = False
    if is_bounds:
        while not is_valid:
            val = click.prompt('Enter %s coordinate between %.2f-%.2f or q to exit' % (var_name,min_in,max_in))
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
                rospy.loginfo('Enter number between %.2f-%.2f' % (min_in,max_in))
    else:
        while not is_valid:
            val = click.prompt('Enter %s coordinate' % var_name)
            try:
                out_val = float(val)
                is_valid = True
            except:
                rospy.loginfo('Enter valid number characters')
    
    rospy.loginfo('entered %.2f' % out_val)
    return out_val


if __name__ == '__main__':
    main_swim()
