#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import Twist
import click


def control_turtle_keyboard():
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.init_node('teleop_node')
    r = rospy.Rate(10)

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

if __name__ == '__main__':
    try:
        control_turtle_keyboard()
    except:
        rospy.loginfo("End of Control")

