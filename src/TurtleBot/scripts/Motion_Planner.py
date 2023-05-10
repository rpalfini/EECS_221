#!/usr/bin/env python2

import rospy
from TurtleBot.msg import Reference_Pose, PID_Gains
import swim_to_goal as util

def request_mode():
    return util.request_number('mode',bounds=(0,1))

def request_ref_point():
    x = util.request_number('x',is_bounds=False)
    y = util.request_number('y',is_bounds=False)
    theta = util.request_number('theta (radians)',is_bounds=False)
    return (x,y,theta)

def create_ref_msg(mode,ref_tuple):
    target_pose = Reference_Pose()
    target_pose.mode = mode
    target_pose.pose.x = ref_tuple[0]
    target_pose.pose.y = ref_tuple[1]
    target_pose.theta = ref_tuple[2]

def is_goal_state():
    # checks if robot is in final position, x,y,theta
    goal_check = True
    if goal_check:
        return True
    else:
        return False

def main():
    is_user_input = True
    r = rospy.Rate(10)
    pub_ref_pose = rospy.Publisher('/reference_pose', Reference_Pose, queue_size=5)
    # pub_pos_gains = rospy.Publisher('/pos_gains', PID_Gains, queue_size=5)
    # pub_ang_gains = rospy.Publisher('/ang_gains', PID_Gains, queue_size=5)
    ref_pose = create_ref_msg()
    while not rospy.is_shutdown():
        while not rospy.is_shutdown() and not is_goal_state()
        r.sleep()


if __name__ == "__main__":
    main()