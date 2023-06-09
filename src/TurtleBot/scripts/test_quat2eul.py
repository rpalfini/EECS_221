#!/usr/bin/env python2
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, Twist, Point, Quaternion, Vector3
from tf.transformations import euler_from_quaternion
import PID_Controller as pid
from TurtleBot.msg import Reference_Pose, PID_Gains, cur_pose

# Create a new model state message object
# model_state_msg = ModelStates()


# # Fill in the pose
# model_state_msg.pose.position = Point(2.0, 3.0, 4.0)
# model_state_msg.pose.orientation = Quaternion(1.0, 20.0, 40.0, 1.0)

# # Fill in the twist
# model_state_msg.twist.linear = Vector3(0.5, 0.0, 0.0)
# model_state_msg.twist.angular = Vector3(0.0, 0.0, 0.5)

# quaternion = [model_state_msg.pose.orientation.x, model_state_msg.pose.orientation.y,
#               model_state_msg.pose.orientation.z, model_state_msg.pose.orientation.w]
# euler = euler_from_quaternion(quaternion)
# print "Roll:", euler[0]
# print "Pitch:", euler[1]
# print "Yaw:", euler[2]

def callback(msg):
    poses = msg.pose
    # Print the poses of the models
    for i, pose in enumerate(poses):
        print('start')
        print("Model", i+1, "Pose:")
        print("Position: ", pose.position)
        print("Orientation: ", pose.orientation)
        print("")

if __name__ == "__main__":
    rospy.init_node('test_subscriber')

    # rospy.Subscriber('/gazebo/model_states', ModelStates, callback)

    # rospy.spin()
    pos_node = pid.SubscriberNode(topic='/gazebo/model_states',msg = ModelStates,msg_object = ModelStates())
    while not rospy.is_shutdown():
        # poses = pos_node.data.pose
        # print(type(poses))
        # print(len(poses))
        # if len(poses) != 0:
        #     print(poses[1])
        
        # print(pos_node.data.pose)
        euler = pid.format_model_state(pos_node)
        print "Yaw:", euler["theta"]
        # Print the Euler angles


