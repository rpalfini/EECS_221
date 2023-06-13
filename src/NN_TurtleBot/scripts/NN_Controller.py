#!/usr/bin/env python2

import rospy
import numpy as np
import matplotlib.pyplot as plt
import plot_utils
from std_msgs.msg import Float64MultiArray
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
import keras as K
from keras.models import load_model
import tensorflow as tf

class SubscriberNode(object):
    def __init__(self,topic,msg,msg_object):
        self.data = msg_object
        self.prev_data = None
        rospy.Subscriber(topic, msg, self.callback)

    def callback(self, data):
        self.prev_data = self.data
        self.data = data

    def is_received(self):
        if self.prev_data is None:
            return False
        else:
            return True

def main():
    rospy.init_node('NN_controller_node')

    #setup subscribers
    ref_pose_node = SubscriberNode('/reference_pose',Float64MultiArray,Float64MultiArray())
    pos_node = SubscriberNode(topic='/gazebo/model_states',msg = ModelStates,msg_object = ModelStates())

    #setup publishers
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    r = rospy.Rate(10)

    #load model
    # model_path = '/home/rpalfini/Palfini_Robert_ws/model_e20_b32_l20_n128.h5'
    model_path = 'model_e20_b32_l20_n128.h5'
    loaded_model = tf.keras.models.load_model(model_path,compile=False)

    # wait for model state to load


    # wait for ref_msg



    is_first_msg = True
    while not rospy.is_shutdown():
        if at_goal():
            if is_first_msg:
                rospy.loginfo('arrived at goal')
                is_first_msg = False
        else:
            is_first_msg = True
            nn_input = make_nn_input(pos_node,ref_pose_node)
            u = loaded_model.predict(np.array([nn_input]))[0]


def make_cmd_msg(u):
    move_cmd = Twist()
    move_cmd.linear.x = u[0]
    move_cmd.angular.z = u[1]
    return move_cmd

def make_nn_input(pos_node,ref_pose_node):
    cur_state = format_model_state(pos_node)
    pose = cur_state['pose']
    theta = cur_state['theta']
    ref_pose = ref_pose_node.data.data
    nn_input = np.array([pose.x, pose.y, theta, ref_pose[0], ref_pose[1]])
    return nn_input


def at_goal():
    return False


def find_model_index(pos_node):
    model_names = ['turtlebot3','turtlebot3_burger']
    names = pos_node.data.name
    model_idx = []
    for name in model_names:
        if name in names:
            model_idx.append(names.index(name))
    if not len(model_idx) == 1:
        raise Exception("More than one model matched model_name")
    return model_idx[0]

def format_model_state(node):
    # this expects node of type gazebo_msg/model_state
    if len(node.data.pose) == 0:
        pose_val_out = Point()
        euler_angs = [0,0,0]
    else:
        model = find_model_index(node)
        pose_val_out = node.data.pose[model].position
        euler_angs = quaternion_to_euler(node.data.pose[model].orientation)
    
    ref_dict = {'pose': pose_val_out, 'theta': euler_angs[2]} # euler_angs[2] corresponds to the yaw
    return ref_dict

def quaternion_to_euler(orientation):
    # takes quaternion object and converts it euler angles
    quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
    euler = euler_from_quaternion(quaternion)
    return euler

if __name__ == "__main__":
    main()
