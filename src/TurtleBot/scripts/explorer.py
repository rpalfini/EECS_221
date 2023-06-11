#!/usr/bin/env python2

import rospy
import numpy as np
import matplotlib.pyplot as plt
import random
import swim_to_goal2 as util
import PID_Controller as pid
import rrt
import Motion_Planner as mp
from std_msgs.msg import Float64MultiArray
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import OccupancyGrid

def main():
    global robot_radius
    global use_dilated_map
    rospy.init_node('explorer_node')
    
    args = arg_parse()
    sub_nodes = create_subscribers()
    map_node = sub_nodes['map_node']
    pos_node = sub_nodes['pos_node']

    # set up publishers
    next_point_pub = rospy.Publisher('/target_pose',Float64MultiArray,queue_size=5)
    r = rospy.Rate(10)

    # flag for one time messages
    is_first = True
    is_first_point_recorded = False
    plot_next_points = True
    if plot_next_points:
        plt.figure()
        plt.ion()
        plt.title('Exploration Points')
        plt.show()
    
    # load_map before executing node
    rrt.load_map(map_node)    

    # search params
    explore_radius_real = 0.5 # meters
    explore_radius_index = explore_radius_real/map_node.data.info.resolution # pixels
    
    cur_map = rrt.process_map(map_node.current_map)

    # if plot_next_points:
    #     plt.imshow(cur_map,cmap='gray')
    #     plt.scatter(*current_point_index)

    while not is_explored(cur_map[:,:,0]) and not rospy.is_shutdown():
        is_arrived = False
        is_first = True

        current_point_real = get_point_xy(pos_node)
        current_point_index = rrt.convert_traj_to_index(current_point_real,map_node.data.info)
        if not is_first_point_recorded:
            very_first_point = current_point_index
            is_first_point_recorded = True
        


        # next_point_index = select_next_point_weighted(cur_map[:,:,0],current_point_index,explore_radius_index)
        if get_true_percentage(5):
            next_point_index = very_first_point
            rospy.loginfo('first_point_commanded')
        else:
            next_point_index = select_next_point_weighted(map_node.current_map,current_point_index,explore_radius_index)
        # next_point_index = select_next_point_random(cur_map[:,:,0])
        # next_point_index = select_next_point_random_radius(cur_map[:,:,0],current_point_index,explore_radius_index)
        
        if plot_next_points:
            plt.imshow(cur_map,cmap='gray')
            plt.scatter(*next_point_index)
            plt.pause(0.5)

        next_point = rrt.convert_traj_to_real(next_point_index,map_node.data.info)
        
        rospy.loginfo('Selected next point (%.2f,%.2f)' % (next_point[0],next_point[1]))
        while not is_arrived and not rospy.is_shutdown():
            is_first = mp.status_msg('Sent point to motion planner',is_first)
            target_pose = make_target_pose_msg(next_point)
            next_point_pub.publish(target_pose)
            r.sleep()

            if util.check_if_arrived(pid.format_model_state(pos_node),[next_point[0],next_point[1]],pos_err=0.4):
                is_arrived = True

        cur_map = rrt.process_map(map_node.current_map)
    rospy.loginfo('map fully explorered')

def get_true_percentage(percent):
    return random.random() < percent/100.0

def get_point_xy(pos_node):
    formatted_node = pid.format_model_state(pos_node)
    point_data = formatted_node['pose']
    point_out = [point_data.x, point_data.y]
    return point_out

def make_target_pose_msg(next_point):
    msg_out = Float64MultiArray()
    msg_out.data = [next_point[0],next_point[1]]
    return msg_out

def arg_parse():
    args = {}
    args['robot_radius'] = rospy.get_param('~robot_radius',3.7)
    args['use_dilated_map'] = rospy.get_param('~use_dilated_map',True)

def create_subscribers():
    sub_nodes = {}
    sub_nodes['map_node'] = rrt.SubscriberNode_Map('/map',OccupancyGrid,OccupancyGrid())
    sub_nodes['pos_node'] = pid.SubscriberNode(topic='/gazebo/model_states',msg = ModelStates,msg_object = ModelStates())
    return sub_nodes

def select_next_point_random(map_array):
    free_indices = np.where(map_array > 250)
    free_points = np.column_stack(free_indices)
    selected_index = np.random.choice(len(free_indices))
    selected_point = free_points[selected_index]
    selected_point = [selected_point[1], selected_point[0]]
    return selected_point

def select_next_point_random_radius(map_array, current_point, radius):
    # picks a point randomly within a radius
    current_x, current_y = current_point
    current_point = [current_y,current_x]

    free_indices = np.where(map_array > 250)
    free_points = np.column_stack(free_indices)

    diff = free_points - current_point
    distances = np.linalg.norm(diff, axis=1)

    valid_indices = np.where(distances <= radius)[0]
    valid_free_points = free_points[valid_indices]

    selected_index = np.random.choice(len(valid_indices))
    selected_point = free_points[selected_index]
    selected_point = [selected_point[1], selected_point[0]] # flip back to x y coordinate
    return selected_point

def select_next_point_weighted(map_array, current_point, radius):
    height, width = map_array.shape
    current_x, current_y = current_point
    current_point = [current_y,current_x] # flip from to y,x as this is how the map array is indexed

    unexplored_indices = np.where(map_array == -1)
    unexplored_points = np.column_stack(unexplored_indices)

    free_indices = np.where(map_array == 0)
    free_points = np.column_stack(free_indices)

    # Calculate the distances between free points and unexplored points
    diff = free_points - current_point
    distances = np.linalg.norm(diff, axis=1)

    # Filter free points that are within the radius of the current point
    valid_indices = np.where(distances <= radius)[0]
    valid_free_points = free_points[valid_indices]

    # Calculate distances between valid free points and unexplored points
    diff = valid_free_points[:, None] - unexplored_points[None, :]
    distances = np.linalg.norm(diff, axis=2)

    # Calculate weights based on distances (Gaussian-based weights)
    weights = np.exp(-distances)
    weights /= np.sum(weights)

    # Flatten the weights array
    weights_flat = weights.flatten()

    # Filter weights_flat to match the valid indices
    valid_weights = weights_flat[valid_indices]

    # Rescale valid_weights so that their sum is 1
    valid_weights /= np.sum(valid_weights)

    # Select the next point randomly based on the weights
    selected_index = np.random.choice(len(valid_free_points), p=valid_weights)
    selected_point = valid_free_points[selected_index]

    selected_point = [selected_point[1], selected_point[0]] # flip back so coordinates are x,y, not y,x

    return selected_point

def is_explored(map_array):
    # checks if every point on the map has been explored
    zero_indices = np.where(map_array == 0)
    zero_points = np.column_stack(zero_indices)

    for point in zero_points:
        if map_array[point[0], point[1]] == 100:
            return True

    return False

if __name__ == "__main__":
    main()