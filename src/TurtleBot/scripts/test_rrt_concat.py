#!/usr/bin/env python2

import rrt
import rospy
from nav_msgs.msg import OccupancyGrid


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
        print(len(map_node.data.data))