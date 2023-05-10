#!/usr/bin/env python2
import rospy
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Twist, Point, Quaternion, Vector3
from tf.transformations import euler_from_quaternion

# Create a new model state message object
model_state_msg = ModelState()

# Fill in the model name
model_state_msg.model_name = "my_robot"

# Fill in the pose
model_state_msg.pose.position = Point(2.0, 3.0, 4.0)
model_state_msg.pose.orientation = Quaternion(1.0, 20.0, 40.0, 1.0)

# Fill in the twist
model_state_msg.twist.linear = Vector3(0.5, 0.0, 0.0)
model_state_msg.twist.angular = Vector3(0.0, 0.0, 0.5)

quaternion = [model_state_msg.pose.orientation.x, model_state_msg.pose.orientation.y,
              model_state_msg.pose.orientation.z, model_state_msg.pose.orientation.w]
euler = euler_from_quaternion(quaternion)

# Print the Euler angles
print("Roll:", euler[0])
print("Pitch:", euler[1])
print("Yaw:", euler[2])

