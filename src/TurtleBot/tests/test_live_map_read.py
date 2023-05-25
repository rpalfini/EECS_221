#!/usr/bin/env python2

import rospy
import numpy as np
from matplotlib import cm
import matplotlib.pyplot as plt
from scipy.misc import imread
import random, sys, math, os.path
import cv2

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float64MultiArray
import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../scripts")

import PID_Controller as pid
import Motion_Planner as mp
import rrt
import pickle


def plot_map(map_data):
    # expects mapp as np array
    plt.imshow(map_data,cmap='gray')
    plt.show()


def main():
    plt.figure()
    plt.ion()
    rospy.init_node('Map_Explorer_Node')
    map_node = rrt.SubscriberNode_Map('/map',OccupancyGrid,OccupancyGrid())
    while not map_node.is_received() and not rospy.is_shutdown():
        rospy.sleep(0.5)
    with open('map_node.pkl','wb') as f:
        output = {'map': map_node.current_map}
        # output = {'map': 5}


        print(type(map_node.current_map))
        pickle.dump(output,f,protocol=2)
    while not rospy.is_shutdown():
        plot_map(map_node.current_map)
        rospy.loginfo('map updated')
        rospy.sleep(2)
    


if __name__ == "__main__":
    main()
    
