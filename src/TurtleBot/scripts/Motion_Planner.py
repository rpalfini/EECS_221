#!/usr/bin/env python2

import rospy
from TurtleBot.msg import Reference_Pose, PID_Gains
import swim_to_goal as util
import PID_Controller as pid
from std_msgs.msg import Float64MultiArray

## for testing mode 1
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
    
# For testing mode 2
def request_start_goal_msg():
    x_start = util.request_number('x_start',is_bounds=False)
    y_start = util.request_number('y_start',is_bounds=False)
    x_goal = util.request_number('x_goal',is_bounds=False)
    y_goal = util.request_number('y_goal',is_bounds=False)
    msg_out = Float64MultiArray()
    msg_out.data = [x_start,y_start,x_goal,y_goal]
    return msg_out

def main():
    rospy.init_node('Motion_Planner')
    testing_problem = 2 # this variable used to specify which problem from miniproject 2 we are trying to test
    is_user_input = True
    is_traj_processed = False
    prev_traj = []
    r = rospy.Rate(10)
    
    if testing_problem == 1:
        pos_node = pid.SubscriberNode(topic='/gazebo/model_states',msg=pid.ModelStates,msg_object=pid.ModelStates())
        pub_ref_pose = rospy.Publisher('/reference_pose', Reference_Pose, queue_size=5)
        ref_pose = request_and_create()
    elif testing_problem == 2:
        start_goal_pub = rospy.Publisher('/start_goal',Float64MultiArray,queue_size=5)
        traj_node = pid.SubscriberNode(topic='/trajectory', msg=Float64MultiArray,msg_object=Float64MultiArray())
        start_goal = request_start_goal_msg()
    
    while not rospy.is_shutdown():
        if testing_problem == 1:
            if is_user_input and is_goal_state(pos_node,ref_pose):
                # mode for testing pid controller
                ref_pose = request_and_create()
            
            while not rospy.is_shutdown() and not is_goal_state(pos_node,ref_pose):
                pub_ref_pose.publish(ref_pose)
                r.sleep()
        if testing_problem == 2:
            if is_traj_processed == True:
                start_goal = request_start_goal_msg()
                is_traj_processed = False
            
            start_goal_pub.publish(start_goal)
            r.sleep()
            if not traj_node.data.data == [] and not traj_node.data.data == prev_traj:
                rospy.loginfo('Trajectory Received for start (%.2f,%.2f) and end (%.2f,%.2f)' % (start_goal.data[0],start_goal.data[1],start_goal.data[2],start_goal.data[3]))
                rospy.loginfo(str(traj_node.data.data))
                prev_traj = traj_node.data.data
                is_traj_processed = True

        


if __name__ == "__main__":
    main()