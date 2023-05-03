#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from TurtleBot.msg import Reference_Pose
import swim_to_goal as util

class SubscriberNode(object):
    def __init__(self,topic,msg):
        self.data = None
        self.prev_data = None
        self.last_callback_time = 0
        self.time_since_last_callback = None
        rospy.Subscriber(topic, msg, self.callback)

        self.not_first_callback = False

    def callback(self, data):
        self.prev_data = self.data
        self.data = data
        callback_time = rospy.get_time()
        if self.not_first_callback:
            self.time_since_last_callback = callback_time - self.last_callback_time
        self.last_callback_time = callback_time

class PID_gains(object):
    def __init__(self,Kp,Ki,Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

def pid_ang():
    Kp = 5
    Ki = 0
    Kd = 0

def pid(Kp,Ki,Kd):
    return Kp

def accumulate_error(prev_total_err,new_err):
    error_max = 100
    total_err = prev_total_err + new_err
    if total_err > error_max:
        total_err = error_max
    elif total_err < error_max:
        total_err = - error_max
    return total_err

def derivative_error(prev_err,new_err,dt):
    return (new_err - prev_err)/dt

def format_target(node):
    return (node.data.pose.x, node.data.pose.y)

def main():
    rospy.init_node('PID_Controller')
    # setup subscriptions
    pos_node = SubscriberNode(topic='/turtle1/pose',msg = Pose)
    ref_node = SubscriberNode(topic='/reference_pose',msg = Reference_Pose)
    # setup publisher
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    r = rospy.Rate(40)
    move_cmd = Twist()
    
    while not rospy.is_shutdown():
        is_arrived = util.check_if_arrived(pos_node.data,format_target(ref_node))
        if not is_arrived:
            if ref_node.data.mode == 0:
                # first turn to face target, then move to target
                err_pos, err_ang = util.calc_error(pos_node.data,format_target(ref_node))
                is_facing_target = util.check_body_angle(pos_node.data,format_target(ref_node))
                while not is_facing_target and not rospy.is_shutdown():
                    move_cmd = Twist()
                    # turn to face target
                    is_facing_target = util.check_body_angle(pos_node.data,format_target(ref_node))
                    err_pos,err_ang = util.calc_error(pos_node.data,format_target(ref_node))
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
                    is_arrived = util.check_if_arrived(pos_node.data,format_target(ref_node))
                    err_pos,err_ang = calc_error(pos_node.data,format_target(ref_node))
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

            elif ref_node.data.mode == 1:
                # turn and move to target at the same time

        
    


if __name__ == "__main__":
    main()
