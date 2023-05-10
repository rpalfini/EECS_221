#!/usr/bin/env python2

import rospy
from TurtleBot.msg import PID_Gains
import PID_Controller as pid

pose1 = PID_Gains(Kp=1,Ki=3,Kd=3)
pose2 = PID_Gains(Kp=1,Ki=2,Kd=3)

answer = pid.is_msg_same(pose1,pose2)
print(answer)