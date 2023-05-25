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
    rospy.init_node('Map_Explorer_Node')
    map_node = rrt.SubscriberNode_Map('/map',OccupancyGrid,OccupancyGrid())
    with open('map_node.pickle','wb') as f:
        f.dump({'map': map_node.current_map},f)
    plot_map(map_node.current_map)







if __name__ == "__main__":
    main()
    
