#!/usr/bin/env python2

import rospy
import numpy as np
from matplotlib import cm
import matplotlib.pyplot as plt
from scipy.misc import imread
import random, sys, math, os.path
import cv2
import pickle
import click

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float64MultiArray
import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../scripts")

import PID_Controller as pid
import Motion_Planner as mp
import rrt


def create_subscribers():
    sub_nodes = {}
    sub_nodes['map_node'] = rrt.SubscriberNode_Map('/map',OccupancyGrid,OccupancyGrid())
    return sub_nodes

def arg_parse():
    args = {}
    args['is_real'] = rospy.get_param('~is_real',True)
    return args

def main():
    rospy.init_node('test_map_query')
    print('one')
    sub_nodes = create_subscribers()
    print('two')
    rrt.load_map(sub_nodes['map_node'])
    args = arg_parse()
    input_is_real = args['is_real']
    print('three')
    
    while not rospy.is_shutdown():
        x_origin = sub_nodes['map_node'].data.info.origin.position.x
        y_origin = sub_nodes['map_node'].data.info.origin.position.y
        resolution = sub_nodes['map_node'].data.info.resolution
        point_in = mp.request_ref_point(request_theta=False)
        if input_is_real:
            print('input is real')
            point2check = [rrt.convert_real_to_index(point_in[0],x_origin,resolution), rrt.convert_real_to_index(point_in[1],y_origin,resolution)]
            print('Point (%d,%d) is %d' % (point2check[0],point2check[1],rrt.query_map_cell_value(sub_nodes['map_node'],point2check[0],point2check[1])))
        else:
            print('input is index')
            point2check = [int(x) for x in point_in]
            print('Point (%d,%d) is %s' % (point2check[0],point2check[1],rrt.query_map_cell_value(sub_nodes['map_node'],point2check[0],point2check[1])))
        
        

if __name__ == "__main__":
    main()
