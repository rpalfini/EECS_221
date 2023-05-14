#!/usr/bin/env python2

import rospy
from TurtleBot.msg import Reference_Pose, PID_Gains
import swim_to_goal as util
import PID_Controller as pid
from std_msgs.msg import Float64MultiArray
from gazebo_msgs.msg import ModelStates

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
    prev_target_pose = []
    target_received = False
    mode = 0
    msg_count = 1
    r = rospy.Rate(10)
    
    if testing_problem == 1: #testing PID Controller
        pos_node = pid.SubscriberNode(topic='/gazebo/model_states',msg=pid.ModelStates,msg_object=pid.ModelStates())
        pub_ref_pose = rospy.Publisher('/reference_pose', Reference_Pose, queue_size=5)
        ref_pose = request_and_create()
    elif testing_problem == 2: #testing RRT 
        start_goal_pub = rospy.Publisher('/start_goal',Float64MultiArray,queue_size=5)
        traj_node = pid.SubscriberNode(topic='/trajectory', msg=Float64MultiArray,msg_object=Float64MultiArray())
        start_goal = request_start_goal_msg()
    elif testing_problem == 3: #full stack
        pub_ref_pose = rospy.Publisher('/reference_pose', Reference_Pose, queue_size=5)
        start_goal_pub = rospy.Publisher('/start_goal',Float64MultiArray,queue_size=5)
        traj_node = pid.SubscriberNode(topic='/trajectory', msg=Float64MultiArray,msg_object=Float64MultiArray())
        pos_node = pid.SubscriberNode(topic='/gazebo/model_states',msg = ModelStates,msg_object = ModelStates())
        target_pose_node = pid.SubscriberNode(topic='/target_pose',msg=Float64MultiArray,msg_object=Float64MultiArray())

    while not rospy.is_shutdown():
        if testing_problem == 1:
            if is_user_input and is_goal_state(pos_node,ref_pose):
                # mode for testing pid controller
                ref_pose = request_and_create()
            
            while not rospy.is_shutdown() and not is_goal_state(pos_node,ref_pose):
                pub_ref_pose.publish(ref_pose)
                r.sleep()
        elif testing_problem == 2:
            if is_traj_processed == True:
                start_goal = request_start_goal_msg()
                is_traj_processed = False
            
            start_goal_pub.publish(start_goal)
            r.sleep()
            # if not traj_node.data.data == [] and not traj_node.data.data == prev_traj:
            if is_new_msg(traj_node,prev_traj):
                # rospy.loginfo('Trajectory Received for start (%.2f,%.2f) and end (%.2f,%.2f)' % (start_goal.data[0],start_goal.data[1],start_goal.data[2],start_goal.data[3]))
                log_rec_traj(start_goal)
                rospy.loginfo(str(traj_node.data.data))
                prev_traj = traj_node.data.data
                is_traj_processed = True
        elif testing_problem == 3:
            while not target_received:
                # if not target_pose_node.data.data == [] and not target_pose_node.data.data == prev_traj:
                if is_new_msg(target_pose_node,prev_target_pose):
                    log_rec_traj(target_pose_node)
                    prev_target_pose = target_pose_node.data.data
                    target_received = True
                    start_goal_msg = make_start_goal_msg(target_pose_node)
                    start_goal_pub.publish(start_goal_msg)
                    r.sleep()
            while not is_traj_processed:
                if is_new_msg(traj_node,prev_traj):
                    log_rec_traj(start_goal)
                    rospy.loginfo(str(traj_node.data.data))
                    prev_traj = traj_node.data.data
                    is_traj_processed = True
            traj = traj_node.data.data
            for ii in range(0,len(traj)-1,2):
                rospy.loginfo('sending point number %d' % (msg_count))
                msg_count += 1
                next_point = (traj[ii],traj[ii+1])
                next_angle = find_angle_next_point(cur_location,next_point)
                ref_pose = create_ref_msg(mode=mode,ref_tuple=(*next_point,next_angle))
                while not at_next_location(pos_node,ref_pose):

                    pub_ref_pose.publish(ref_pose)
                rospy.loginfo('robot arrived at location')
            rospy.loginfo('robot arrived at goal')
            
def find_angle_next_point():
    # finds the angle to next point and adds to ref msg
    return None

def make_start_goal_msg():
    #makes start_goal msg that gets sent to RRT
    return None         



def is_new_msg(data_node,prev_traj):
    if not data_node.data.data == [] and not data_node.data.data == prev_traj:
        return True
    else:
        return False
    
def log_rec_traj(traj_in_node):
    #expect traj_vector to be 4 long list ordered x_start y_start x_goal y_goal
    rospy.loginfo('Trajectory Received for start (%.2f,%.2f) and end (%.2f,%.2f)' % (traj_in_node.data[0],traj_in_node.data[1],traj_in_node.data[2],traj_in_node.data[3]))


if __name__ == "__main__":
    main()