#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion, Pose
from gazebo_msgs.msg import ModelStates
# from turtlesim.msg import Pose
from TurtleBot.msg import Reference_Pose, PID_Gains, err_vals, cur_pose
import swim_to_goal as util
from tf.transformations import euler_from_quaternion
import math
import numpy as np

class SubscriberNode(object):
    def __init__(self,topic,msg,msg_object):
        self.data = msg_object
        self.prev_data = None
        rospy.Subscriber(topic, msg, self.callback)

    def callback(self, data):
        self.prev_data = self.data
        self.data = data

class SubscriberNodeUpdateGains(SubscriberNode):
    def __init__(self, topic, msg,msg_object, gains_obj):
        super(SubscriberNodeUpdateGains, self).__init__(topic, msg, msg_object)
        self.gains_obj = gains_obj

    def callback(self, data):
        super(SubscriberNodeUpdateGains, self).callback(data)
        self.gains_obj.update_gains(data)

# class SubscriberNodeUpdatePose(SubscriberNode):
#     def __init__(self, topic, msg):
#         super().__init__(topic, msg)

#     def callback(self, data):
#         super().callback(data)
#         self.gains_obj.update_pose_info(data)

    # def update_pose_info(data):
    #     # used for Gazebo/model_State input
    #     return None

class PID_gains(object):
    def __init__(self,Kp,Ki,Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def update_gains(self,gains_msg):
        if not gains_msg.Kp == self.Kp and gains_msg.Ki == self.Ki and gains_msg.Kd == self.Kd:
            self.Kp = gains_msg.Kp
            self.Ki = gains_msg.Ki
            self.Kd = gains_msg.Kd            
            debug_info("New Gains:",Kp=self.Kp,Ki=self.Ki,Kd=self.Kd)

class err_struct(object):
    def __init__(self,error_max=100):
        self.err = 0
        self.int_err = 0
        self.deriv_err = 0
        self.prev_err = 0
        self.prev_int_err = 0
        self.error_max = error_max
        self.is_first = True # flag used to show max out msg one
    
    def record_err(self,new_err):
        # dt is the time between measurements
        self.update_err(new_err)
        new_int_err = self.accumulate_error(new_err)
        self.update_int_err(new_int_err)
        self.update_d_error(new_err)

    def update_err(self,new_err):
        self.prev_err = self.err
        self.err = new_err

    def update_int_err(self,new_int_err):
        self.prev_int_err = self.int_err
        self.int_err = new_int_err

    def accumulate_error(self,new_err):
        total_err = self.prev_int_err + new_err
        if total_err > self.error_max:
            total_err = self.error_max
            if self.is_first:
                rospy.loginfo('Upper Error Max Hit %.2f' % (self.error_max))
                self.is_first = False
        elif total_err < self.error_max:
            total_err = -self.error_max
            if self.is_first:
                rospy.loginfo('Lower Error Max Hit %.2f' % (-self.error_max))
                self.is_first = False
        else:
            self.is_first = True
        return total_err

    def update_d_error(self,new_err):
        self.deriv_err = (new_err - self.prev_err)

    def make_err_val_msg(self):
        out_msg = err_vals()
        out_msg.err = self.err 
        out_msg.int_err = self.int_err
        out_msg.deriv_err = self.deriv_err
        out_msg.prev_err = self.prev_err
        out_msg.prev_int_err = self.prev_int_err
        return out_msg

def pid(gains,err_struct):
    return gains.Kp*err_struct.err + gains.Ki*err_struct.int_err + gains.Kd*err_struct.deriv_err

def main():
    rospy.init_node('PID_Controller')
    # test_mode specifies if gains are hard coded or received from topic
    test_mode = True
    # setup subscriptions
    pos_node = SubscriberNode(topic='/gazebo/model_states',msg = ModelStates,msg_object = ModelStates())
    ref_node = SubscriberNode(topic='/reference_pose',msg = Reference_Pose,msg_object = Reference_Pose())
    
    # setup publisher
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    ang_pub_err = rospy.Publisher('/ang_err', err_vals, queue_size=5)
    pos_pub_err = rospy.Publisher('/pos_err', err_vals,queue_size=5)
    # pub_rec_pose = rospy.Publisher('/rec_pose', cur_pose, queue_size=5)
    r = rospy.Rate(40)
    
    dbg = True
    debug_interval = 20
    
    pos_gains = PID_gains(Kp=1.5,Ki=0,Kd=0)
    ang_gains = PID_gains(Kp=2,Ki=0,Kd=0)
    pos_err = err_struct()
    ang_err = err_struct(error_max=10)
    
    if test_mode:
        pos_gains_node=SubscriberNodeUpdateGains(topic='/pos_gains',msg=PID_Gains,msg_object=PID_Gains(),gains_obj=pos_gains)
        ang_gains_node=SubscriberNodeUpdateGains(topic='/ang_gains',msg=PID_Gains,msg_object=PID_Gains(),gains_obj=ang_gains)

    iterations=0 # used for debugging

    while not rospy.is_shutdown():
        if not util.check_if_arrived(format_model_state(pos_node),format_target(ref_node)):
            activate_controller(pos_node, ref_node, pub, pos_pub_err, ang_pub_err, r, dbg, debug_interval, pos_gains, ang_gains, pos_err, ang_err, iterations)

## Controller Functions
def activate_controller(pos_node, ref_node, pub, pos_pub_err, ang_pub_err, r, dbg, debug_interval, pos_gains, ang_gains, pos_err, ang_err, iterations):
    is_final_pose = False
    while not is_final_pose and not rospy.is_shutdown():
        is_arrived = util.check_if_arrived(format_model_state(pos_node),format_target(ref_node))
        if not is_arrived:
            if ref_node.data.mode == 0:
                # first turn to face target, then move to target, then adjust to final reference angle
                rospy.loginfo('Using Mode 0')
                turn_to_target(pos_node, ref_node, pub, ang_pub_err, r, dbg, debug_interval, ang_gains, ang_err, iterations)
                move_to_target(pos_node, ref_node, pub, pos_pub_err, r, dbg, debug_interval, pos_gains, pos_err, iterations)
                turn_to_ref_theta(pos_node, ref_node, pub, ang_pub_err, r, dbg, debug_interval, ang_gains, ang_err, iterations)
                is_final_pose = True
                iterations = 0

            elif ref_node.data.mode == 1:
                # turn and move to target at the same time
                rospy.loginfo('Using Mode 1')
                move_and_turn_to_target(pos_node, ref_node, pub, pos_pub_err, ang_pub_err, r, dbg, debug_interval, ang_gains, ang_err, pos_gains, pos_err, iterations)
                turn_to_ref_theta(pos_node, ref_node, pub, ang_pub_err, r, dbg, debug_interval, ang_gains, ang_err, iterations)
                is_final_pose = True
                iterations = 0
    rospy.loginfo('Reached Final Pose')
    move_cmd = Twist()
    pub.publish(move_cmd)



def move_and_turn_to_target(pos_node, ref_node, pub, pos_err_node, ang_err_node, r, dbg, debug_interval, ang_gains, ang_err, pos_gains, pos_err, iterations):
    # move and turn to target
    rospy.loginfo('Moving And Turning To Target')
    while not util.check_if_arrived(format_model_state(pos_node),format_target(ref_node)) and not rospy.is_shutdown():
        
        move_cmd = Twist()
        err_pos, err_ang = util.calc_error(format_model_state(pos_node),format_target(ref_node))
        ang_err.record_err(err_ang)
        pos_err.record_err(err_pos) 
        pos_err_msg = pos_err.make_err_val_msg()
        ang_err_msg = ang_err.make_err_val_msg()
        move_cmd.angular.z = pid(ang_gains,ang_err)
        move_cmd.linear.x = pid(pos_gains, pos_err)
        
        # if is_debug_iter(dbg,debug_interval,iterations):
        #     debug_info("Position:",err=pos_err.err,int_err=pos_err.int_err,d_error=pos_err.deriv_err,prev_err=pos_err.prev_err,prev_int_err=pos_err.prev_int_err)
        #     debug_info("Angular:",err=ang_err.err,int_err=ang_err.int_err,d_error=ang_err.deriv_err,prev_err=ang_err.prev_err,prev_int_err=ang_err.prev_int_err)
        
        pos_err_node.publish(pos_err_msg)
        ang_err_node.publish(ang_err_msg)
        pub.publish(move_cmd)
        iterations += 1
        r.sleep()
    pub.publish(Twist())
    r.sleep()

def turn_to_ref_theta(pos_node, ref_node, pub, pub_err, r, dbg, debug_interval, ang_gains, ang_err, iterations):
    rospy.loginfo('Turning To Final Reference Pose')
    while not is_final_angle(format_model_state(pos_node),ref_node.data.theta) and not rospy.is_shutdown():
        #face final reference pose
        
        move_cmd = Twist()
        # _, err_ang = util.calc_error(format_model_state(pos_node),format_target(ref_node))
        cur_state = format_target(ref_node)
        err_ang = ref_node.data.theta - cur_state['theta']
        ang_err.record_err(err_ang)
        err_msg = ang_err.make_err_val_msg()
        move_cmd.angular.z = pid(ang_gains,ang_err)

        # if is_debug_iter(dbg,debug_interval,iterations):
        #     debug_info("Angular:",err=ang_err.err,int_err=ang_err.int_err,d_error=ang_err.deriv_err,prev_err=ang_err.prev_err,prev_int_err=ang_err.prev_int_err)
        
        pub_err.publish(err_msg)
        pub.publish(move_cmd)
        iterations += 1
        r.sleep()
    pub.publish(Twist())
    r.sleep()

def turn_to_target(pos_node, ref_node, pub, pub_err, r, dbg, debug_interval, ang_gains, ang_err, iterations):
    rospy.loginfo('Turning To Target')
    while not util.check_body_angle(format_model_state(pos_node),format_target(ref_node)) and not rospy.is_shutdown():
        # turn to face target
        
        move_cmd = Twist() #reinitalize move_cmd
        _,err_ang = util.calc_error(format_model_state(pos_node),format_target(ref_node))
        ang_err.record_err(err_ang)
        err_msg = ang_err.make_err_val_msg()
        move_cmd.angular.z = pid(ang_gains,ang_err)

        # if is_debug_iter(dbg,debug_interval,iterations):
        #     debug_info("Calc_err",err_ang=err_ang,err_pos=err_pos)
        #     debug_info("Ang_gains:",Kp=ang_gains.Kp,Ki=ang_gains.Ki,Kd=ang_gains.Kd)
        #     debug_info("Angular:",err=ang_err.err,int_err=ang_err.int_err,d_error=ang_err.deriv_err,prev_err=ang_err.prev_err,prev_int_err=ang_err.prev_int_err)
        
        pub_err.publish(err_msg)
        pub.publish(move_cmd)
        iterations += 1
        r.sleep()
    pub.publish(Twist())
    r.sleep()

def move_to_target(pos_node, ref_node, pub, pub_err, r, dbg, debug_interval, pos_gains, pos_err, iterations):
    rospy.loginfo('Moving To Target')
    while not util.check_if_arrived(format_model_state(pos_node),format_target(ref_node)) and not rospy.is_shutdown(): 
        # move to target
        
        move_cmd = Twist()
        err_pos,_ = util.calc_error(format_model_state(pos_node),format_target(ref_node))
        pos_err.record_err(err_pos)
        err_msg = pos_err.make_err_val_msg()
        move_cmd.linear.x = pid(pos_gains, pos_err)

        # if is_debug_iter(dbg,debug_interval,iterations):
        #     debug_info("Pos_gains:",Kp=pos_gains.Kp,Ki=pos_gains.Ki,Kd=pos_gains.Kd)
        #     debug_info("Position:",err=pos_err.err,int_err=pos_err.int_err,d_error=pos_err.deriv_err,prev_err=pos_err.prev_err,prev_int_err=pos_err.prev_int_err)
        
        pub_err.publish(err_msg)
        pub.publish(move_cmd)
        iterations += 1
        r.sleep()
    pub.publish(Twist())
    r.sleep()

def is_final_angle(cur_state,ref_theta):
    # checks if body is at final reference angle
    
    err_ang = ref_theta - cur_state['theta']
    debug_info('is_final_angle',ref_theta=ref_theta,cur_theta=cur_state['theta'])
    if abs(err_ang) < math.pi/180:
        result = True
        rospy.loginfo('facing final angle')
    else:
        result = False
    return result

## Utility Functions
def is_msg_same(msg1,msg2):
    '''compares if two messages are the same or different'''
    fields = msg1.__slots__
    for field in fields:
        if getattr(msg1,field) != getattr(msg2,field):
            return False
    return True       

def format_target(node):
    # expects Reference message
    return (node.data.point.x, node.data.point.y)

def format_model_state(node):
    # this expects node of type gazebo_msg/model_state
    model = 1
    if len(node.data.pose) == 0:
        pose_val_out = Point()
        euler_angs = [0,0,0]
    else:
        pose_val_out = node.data.pose[model].position
        euler_angs = quaternion_to_euler(node.data.pose[model].orientation)
    
    ref_dict = {'pose': pose_val_out, 'theta': euler_angs[2]} # euler_angs[2] corresponds to the yaw
    return ref_dict

# def format_model_state(data):
#     # this expects node of type gazebo_msg/model_state
#     euler_angs = quaternion_to_euler(data.pose.orientation)
#     ref_dict = {'pose': data.pose.position, 'theta': euler_angs[2]} # euler_angs[2] corresponds to the yaw
#     return ref_dict

def quaternion_to_euler(orientation):
    # takes quaternion object and converts it euler angles
    quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
    euler = euler_from_quaternion(quaternion)
    return euler

def is_debug_iter(dbg_state,interval,iterations):
    if dbg_state and iterations % interval == 0:
        return True
    else:
        return False

def debug_info(info,**kwargs):
    output_string = info + " "
    for key,value in kwargs.iteritems():
        output_string += str(key) + ": " + str(value) + ','
    rospy.loginfo(output_string[:-2])

if __name__ == "__main__":
    main()
