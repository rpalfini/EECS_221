#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from TurtleBot.msg import Reference_Pose

class SubscriberNode(object):
    def __init__(self,topic,msg):
        self.data = None
        rospy.Subscriber(topic, msg, self.callback)

    def callback(self, data):
        self.data = data


def main():
    rospy.init_node('PID_Controller')
    # setup subscriptions
    position_node = SubscriberNode(topic='/turtle1/pose',msg = Pose)
    ref_node = SubscriberNode(topic='/reference_pose',msg = Reference_Pose)
    # setup publisher
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    


if __name__ == "__main__":
    main()
