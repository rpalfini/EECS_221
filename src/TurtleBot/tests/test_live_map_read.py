#!/usr/bin/env python2

import rospy
import numpy as np
from matplotlib import cm
import matplotlib.pyplot as plt
from scipy.misc import imread
import random, sys, math, os.path
import cv2
import pickle

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float64MultiArray
import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../scripts")

import PID_Controller as pid
import Motion_Planner as mp
import rrt


def adjust_map_colors(map_in):
    # replaces the map colors with easier to see values
    # map_out = np.where(map_in == 100,0, map_in)
    map_out = np.where(map_in == 0, 100, np.where(map_in == 100, 0, np.where(map_in == -1, 50, map_in)))
    return map_out


def plot_map(map_data):
    # expects map_data as np array
    # map_data = np.where(map_data == -1,50,map_data) #distinguishes 
    # map_data = adjust_map_colors(map_data)
    # map_data = rrt.map_img(map_data)
    plt.imshow(map_data,cmap='gray')
    # plt.show()


def main():
    plt.figure()
    plt.ion()
    rospy.init_node('Map_Explorer_Node')
    map_node = rrt.SubscriberNode_Map('/map',OccupancyGrid,OccupancyGrid())
    while not map_node.is_received() and not rospy.is_shutdown():
        rospy.sleep(0.5)
    # with open('map_node.pkl','wb') as f:
    #     output = {'map': map_node.current_map}
    #     # output = {'map': 5}

    #     print(type(map_node.current_map))
    #     pickle.dump(output,f,protocol=2)
    
    while not rospy.is_shutdown():
        # if np.array_equal(map_node.prev_map,map_node.current_map):
        #     rospy.loginfo('map is the same')
        # else:
        #     rospy.loginfo('map is different')
        plot_map(map_node.current_map)
        
        rospy.loginfo('map updated')
        num_unknown = np.count_nonzero(map_node.current_map == -1)
        total_cells = map_node.data.info.height*map_node.data.info.width
        percent_unknown = float(num_unknown)/total_cells
        print('%% unknown = %.7f' % percent_unknown)
        print('%d unknown out of %d' % (num_unknown,total_cells))
        plt.pause(0.2)
        plt.show()
    


if __name__ == "__main__":
    main()
    
