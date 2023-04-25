#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import Twist
import click
from turtlesim.msg import Pose
import swim_to_goal as swim


class SubscriberNode(object):
    def __init__(self):
        self.count = 0
        self.data = None
        rospy.Subscriber('/turtle1/pose', Pose, self.callback)
        # rospy.spin()

    def callback(self, data):
        # self.count += 1
        self.data = data

        # if self.count % 20 == 0:
        #     rospy.loginfo('pose %.2f' % data.theta)
        


def control_turtle_keyboard():
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.init_node('teleop_node')
    r = rospy.Rate(1)
    pos_node = SubscriberNode() # receives position feedback


    move_cmd = Twist()
    rospy.loginfo('Starting Keyboard Control (wasd)')
    rospy.loginfo('Press q to quit')
    while not rospy.is_shutdown():
        speed = 2
        ang_speed = 1
        c = click.getchar()
        if c == 'w':
            move_cmd.linear.x = speed 
            move_cmd.angular.z = 0
        elif c == 'a':
            move_cmd.linear.x = 0
            move_cmd.angular.z = ang_speed
        elif c == 'd':
            move_cmd.linear.x = 0
            move_cmd.angular.z = -ang_speed
        elif c == 's':
            move_cmd.linear.x = -speed
            move_cmd.angular.z = 0
        elif c == 'q':
            rospy.signal_shutdown('Node shutting down')
        else:
            move_cmd = Twist()
        pub.publish(move_cmd)
        r.sleep()
        Kp_x = 0.5
        Kp_z = 0.7
    

        rospy.loginfo('x = %.2f; y = %.2f; theta = %.2f' % (pos_node.data.x, pos_node.data.y, pos_node.data.theta))
        target_point = (5,5)
        err_pos,err_ang = swim.calc_error(pos_node.data,target_point)
        lin_vel = Kp_x*err_pos
        ang_vel = Kp_z*err_ang
        move_cmd.linear.x = lin_vel
        move_cmd.angular.z = ang_vel
        rospy.loginfo('err_pos = %.2f; err_ang = %.2f' % (err_pos,err_ang))
        rospy.loginfo('lin_vel = %.2f; ang_vel = %.2f' % (lin_vel,ang_vel))
        
        

if __name__ == '__main__':
    control_turtle_keyboard()


