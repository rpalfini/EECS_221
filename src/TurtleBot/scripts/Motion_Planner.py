#!/usr/bin/env python2

import rospy
import math
from TurtleBot.msg import Reference_Pose, PID_Gains
import swim_to_goal as util
import PID_Controller as pid
from std_msgs.msg import Float64MultiArray
from gazebo_msgs.msg import ModelStates

## for testing problem 1
def request_and_create():
    mode = request_mode()
    ref_pose = request_ref_point()
    return create_ref_msg(mode,ref_pose)

def request_mode():
    return util.request_number('mode',bounds=(0,2))

def request_ref_point(request_theta=True):
    x = util.request_number('x',is_bounds=False)
    y = util.request_number('y',is_bounds=False)
    if request_theta:
        theta = util.request_number('theta (radians)',bounds=(-3.14,3.14))
        return (x,y,theta)
    else:
        return (x,y)

def create_ref_msg(mode,ref_tuple): # also used in testing problem 3
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
    
# For testing problem 2
def request_start_goal_msg():
    x_start = util.request_number('x_start',is_bounds=False)
    y_start = util.request_number('y_start',is_bounds=False)
    x_goal = util.request_number('x_goal',is_bounds=False)
    y_goal = util.request_number('y_goal',is_bounds=False)
    msg_out = Float64MultiArray()
    msg_out.data = [x_start,y_start,x_goal,y_goal]
    return msg_out

# For use in problem 3
def find_angle_next_point(cur_point,next_point):
    # finds the angle to next point and adds to ref msg
    x_err = next_point[0] - cur_point[0]
    y_err = next_point[1] - cur_point[1]
    next_angle = math.atan2(y_err,x_err)
    return next_angle

def make_start_goal_msg(target_pose_node,current_pose_node):
    #makes start_goal msg that gets sent to RRT, RRT expects four number array (x_start,y_start,x_goal,y_goal)
    input_valid = False
    is_first = True
    while not input_valid:
        goal_coord = target_pose_node.data.data
        if len(goal_coord) != 2:
            is_first = status_msg('invalid goal received, send goal of length 2',is_first)
        elif len(goal_coord) == 2:
            input_valid = True
    model_num = pid.find_model_index(current_pose_node)
    x_start = current_pose_node.data.pose[model_num].position.x
    y_start = current_pose_node.data.pose[model_num].position.y
    x_goal = goal_coord[0]
    y_goal = goal_coord[1]
    output = [x_start, y_start, x_goal, y_goal]
    msg = Float64MultiArray()
    msg.data = output
    return msg

def get_cur_xy(pos_node):
    model_num = pid.find_model_index(pos_node)
    x_current = pos_node.data.pose[model_num].position.x
    y_current = pos_node.data.pose[model_num].position.y
    return (x_current,y_current)

def is_new_traj_msg(data_node,prev_traj):
    #TODO this doesnt block when data_node.data.data == [] and not sure why
    if not data_node.data.data == [] and not data_node.data.data == prev_traj:
        return True
    else:
        return False
    
def log_rec_traj(traj_in_node):
    if len(traj_in_node.data) == 2:
        rospy.loginfo('Requested Goal Received (%.2f,%.2f)' % (traj_in_node.data[0],traj_in_node.data[1]))
    else:
        #expect traj_vector to be 4 long list ordered x_start y_start x_goal y_goal
        rospy.loginfo('Trajectory Received for start (%.2f,%.2f) and end (%.2f,%.2f)' % (traj_in_node.data[0],traj_in_node.data[1],traj_in_node.data[2],traj_in_node.data[3]))

def status_msg(msg,is_first):
    if is_first:
        rospy.loginfo(msg)
    return False #use output to turn off is_first

def arg_parse():
    args = {}
    args['test_prob'] = rospy.get_param('~test_prob',3)
    args['mode'] = rospy.get_param('~mode',0)
    return args

def main():
    rospy.init_node('Motion_Planner')
    args = arg_parse()
    testing_problem = args['test_prob'] # this variable used to specify which problem from miniproject 2 we are trying to test
    is_user_input = True # turns on user input for part 1 from command line
    is_traj_processed = False
    prev_traj = []
    prev_target_pose = []
    target_received = False # flag 
    cur_pose_received = False
    mode = args['mode'] # mode to use for part 3
    msg_count = 1
    r = rospy.Rate(10)
    cur_location = 0
    is_first = True #flag for displaying status messages
    also_is_first = True
    
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
            also_is_first = status_msg('start_goal published to rrt, waiting for trajectory',also_is_first)
            r.sleep()
            # if not traj_node.data.data == [] and not traj_node.data.data == prev_traj:
            if is_new_traj_msg(traj_node,prev_traj):
                # rospy.loginfo('Trajectory Received for start (%.2f,%.2f) and end (%.2f,%.2f)' % (start_goal.data[0],start_goal.data[1],start_goal.data[2],start_goal.data[3]))
                log_rec_traj(start_goal)
                rospy.loginfo(str(traj_node.data.data))
                prev_traj = traj_node.data.data
                is_traj_processed = True

        elif testing_problem == 3:
            while not cur_pose_received and not rospy.is_shutdown():
                is_first = status_msg('waiting for model pose',is_first)
                # pid.debug_info('pos_node.isreceived()',is_received=pos_node.is_received())
                if pos_node.is_received():
                    cur_pose_received = True
            is_first = True

            while not target_received and not rospy.is_shutdown():
                is_first = status_msg('waiting for target location',is_first)
                # if not target_pose_node.data.data == [] and not target_pose_node.data.data == prev_traj:
                if is_new_traj_msg(target_pose_node,prev_target_pose):
                    log_rec_traj(target_pose_node.data)
                    prev_target_pose = target_pose_node.data.data
                    target_received = True
                    rospy.loginfo('received following target goal')
                    rospy.loginfo(str(target_pose_node.data.data))
                    start_goal_msg = make_start_goal_msg(target_pose_node,pos_node)
                    start_goal_pub.publish(start_goal_msg)
                    msg_count = 0
                    is_traj_processed = False
                    r.sleep()
            is_first = True

            while not is_traj_processed and not rospy.is_shutdown():
                is_first = status_msg('sent goal location to rrt, waiting for trajectory',is_first)
                start_goal_pub.publish(start_goal_msg)
                r.sleep()
                if is_new_traj_msg(traj_node,prev_traj):
                    log_rec_traj(traj_node.data)
                    rospy.loginfo(str(traj_node.data.data))
                    prev_traj = traj_node.data.data
                    is_traj_processed = True

            traj = traj_node.data.data
            # goal_coord = target_pose_node.data.data
            # if not is_goal_state(pos_node,):
            rospy.loginfo('sending waypoints to controller, total waypoints = %d' % (len(range(0,len(traj)-1,2)))) 
            for ii in range(0,len(traj)-1,2):
                next_point = (traj[ii],traj[ii+1])
                cur_location = get_cur_xy(pos_node)
                next_angle = find_angle_next_point(cur_location,next_point)
                ref_pose = create_ref_msg(mode=mode,ref_tuple=(next_point[0],next_point[1],next_angle))
                rospy.loginfo('sending point number %d at location at (%.2f,%.2f) with angle (%.2f) ' % (msg_count,next_point[0],next_point[1],next_angle))
                msg_count += 1
                while not is_goal_state(pos_node,ref_pose) and not rospy.is_shutdown():
                    pub_ref_pose.publish(ref_pose)
                rospy.loginfo('robot arrived at location (%.2f,%.2f) with angle %.2f radians' % (next_point[0],next_point[1],next_angle))
            rospy.loginfo('robot arrived at goal')
            target_received = False

            


if __name__ == "__main__":
    main()