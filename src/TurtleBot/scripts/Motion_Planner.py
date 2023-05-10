#!/usr/bin/env python2

import rospy
from TurtleBot.msg import Reference_Pose, PID_Gains
import swim_to_goal as util
import PID_Controller as pid

def request_and_create():
    mode = request_mode()
    ref_pose = request_ref_point()
    return create_ref_msg(mode,ref_pose)

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
    target_pose.point.x = ref_tuple[0]
    target_pose.point.y = ref_tuple[1]
    target_pose.theta = ref_tuple[2]
    return target_pose

def format_target_from_msg(ref_pose_msg):
    # requireds Reference_Pose msg
    return [ref_pose_msg.point.x, ref_pose_msg.point.y]

def is_goal_state(pos_node,ref_pose):
    # checks if robot is in final position, x,y,theta
    is_arrived = util.check_if_arrived(pid.format_model_state(pos_node),format_target_from_msg(ref_pose))
    is_ref_angle = pid.is_final_angle(pid.format_model_state(pos_node),ref_pose.theta)
    if is_arrived and is_ref_angle:
        return True
    else:
        return False

def main():
    rospy.init_node('Motion_Planner')
    is_user_input = True
    r = rospy.Rate(10)
    
    pos_node = pid.SubscriberNode(topic='/Gazebo/Model_States',msg=pid.ModelStates,msg_object=pid.ModelStates())
    pub_ref_pose = rospy.Publisher('/reference_pose', Reference_Pose, queue_size=5)
    # pub_pos_gains = rospy.Publisher('/pos_gains', PID_Gains, queue_size=5)
    # pub_ang_gains = rospy.Publisher('/ang_gains', PID_Gains, queue_size=5)
    
    # else:
    #     ref_pose = 

    while not rospy.is_shutdown():
        if is_user_input:
            # mode for testing pid controller
            ref_pose = request_and_create()
            
        while not rospy.is_shutdown() and not is_goal_state(pos_node,ref_pose):
            pub_ref_pose.publish(ref_pose)
            r.sleep()
            


if __name__ == "__main__":
    main()