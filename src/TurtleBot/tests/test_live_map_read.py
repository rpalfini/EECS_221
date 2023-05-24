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
import pickle
import time
import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../scripts")

import PID_Controller as pid
import Motion_Planner as mp
import rrt


if __name__ == "__main__":
    rospy.init_node('Map Explorer')
    
    map_node = rrt.SubscriberNode_Map('/map',)
