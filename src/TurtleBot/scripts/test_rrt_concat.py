#!/usr/bin/env python2

import rrt
import rospy
from nav_msgs.msg import OccupancyGrid

def reverse_calculate(map_node):
    return None


if __name__ == "__main__":
    rrt.rospy.init_node('test_rrt_node')
    map_node = rrt.SubscriberNode_Map(topic='/map',msg=OccupancyGrid,msg_object=OccupancyGrid())
    # while not rospy.is_shutdown():
    #     if len(map_node.data.data) > 0:
    #         print(type(map_node.data.data))
    while not rrt.rospy.is_shutdown():
        if map_node.current_map is None:
            print('map not updated yet')
        else:
            print(map_node.current_map.shape)
            # print('axis1: '+str(len(map_node.current_map[0])))
            # print('axis2: '+str(len(map_node.current_map)))
            # print(map_node.current_map)
            # print(len(map_node.data.data))
    rec_map = False
    while not rospy.is_shutdown() and not rec_map:
        if not map_node.current_map is None:
            rec_map = True

    ## this lets you view the cell values of an individual row
    # row = 384/2
    # for kk in range(384):
    #     print('(%d,%d) has value %d' % (row,kk,map_node.current_map[row,kk]))

    # this lets you see the first and last known values in each row of an image
    # out_file = open('occ_data.txt','w')
    # first_known_found = False
    # for ii in range(384):
    #     for jj in range(384):
    #         if not map_node.current_map[ii,jj] == -1 and not first_known_found:
    #             print >> out_file, 'row %d, first_known = (%d,%d)' % (ii,ii,jj)
    #             first_known_found = True
    #         if map_node.current_map[ii,jj] == -1 and first_known_found:
    #             print >> out_file, 'row %d, last_known = (%d,%d)' % (ii,ii,jj)
    #             first_known_found = False
    #             break

