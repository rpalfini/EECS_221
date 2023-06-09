#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion, Pose
from gazebo_msgs.msg import ModelStates
# from turtlesim.msg import Pose
from TurtleBot.msg import Reference_Pose, PID_Gains, err_vals, cur_pose
import swim_to_goal2 as util
from tf.transformations import euler_from_quaternion
import math
import numpy as np

CTRL_RATE = 0.1
POS_SAT = 0.5
ANG_SAT = 2

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

class SubscriberNode_Repeat(SubscriberNode):
    def __init__(self, topic, msg,msg_object, repeat_topic_node):
        super(SubscriberNode_Repeat, self).__init__(topic, msg, msg_object)
        self.repeat_topic_node = repeat_topic_node

    def callback(self, data):
        super(SubscriberNode_Repeat, self).callback(data)
        self.repeat_topic_node.publish(data)
        
class SubscriberNodeUpdateGains(SubscriberNode):
    def __init__(self, topic, msg,msg_object, gains_obj):
        super(SubscriberNodeUpdateGains, self).__init__(topic, msg, msg_object)
        self.gains_obj = gains_obj

    def callback(self, data):
        super(SubscriberNodeUpdateGains, self).callback(data)
        self.gains_obj.update_gains(data)

class PID_gains(object):
    def __init__(self,Kp,Ki,Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def update_gains(self,gains_msg):
        if not gains_msg.Kp == self.Kp or not gains_msg.Ki == self.Ki or not gains_msg.Kd == self.Kd:
            self.Kp = gains_msg.Kp
            self.Ki = gains_msg.Ki
            self.Kd = gains_msg.Kd            
            debug_info("New Gains:",Kp=self.Kp,Ki=self.Ki,Kd=self.Kd)

class err_struct(object):
    def __init__(self,err_max=100):
        self.err = 0
        self.int_err = 0
        self.deriv_err = 0
        self.prev_err = 0
        self.prev_int_err = 0
        self.error_max = err_max
        self.is_first = True # flag used to show max out msg one
    
    def record_err(self,new_err):
        # dt is the time between measurements
        self.update_err(new_err)
        if have_same_sign(self.err,self.prev_err):
            new_int_err = self.accumulate_error(new_err)
        else:
            new_int_err = 0
        self.update_int_err(new_int_err)
        self.update_d_error(new_err)

    def update_err(self,new_err):
        self.prev_err = self.err
        self.err = new_err

    def update_int_err(self,new_int_err):
        self.prev_int_err = self.int_err
        self.int_err = new_int_err

    def accumulate_error(self,new_err):
        total_err = self.int_err + new_err
        if total_err > self.error_max:
            total_err = self.error_max
            if self.is_first:
                rospy.loginfo('Upper Error Max Hit %.2f' % (self.error_max))
                self.is_first = False
        elif total_err < -self.error_max:
            total_err = -self.error_max
            if self.is_first:
                rospy.loginfo('Lower Error Max Hit %.2f' % (-self.error_max))
                self.is_first = False
        else:
            if not self.is_first:
                rospy.loginfo("No longer at error bound")
            self.is_first = True
        return total_err

    def update_d_error(self,new_err):
        self.deriv_err = (new_err - self.prev_err)
        # self.deriv_err = (self.prev_err - new_err)

    def make_err_val_msg(self,PID_gains):
        out_msg = err_vals()
        out_msg.err = self.err 
        out_msg.int_err = self.int_err
        out_msg.deriv_err = self.deriv_err
        out_msg.prev_err = self.prev_err
        out_msg.prev_int_err = self.prev_int_err
        out_msg.Kp_sig = self.err*PID_gains.Kp
        out_msg.Ki_sig = self.int_err*PID_gains.Ki
        out_msg.Kd_sig = self.deriv_err*PID_gains.Kd
        return out_msg
    
    def reset_int_error(self):
        rospy.loginfo('Int error reset')
        self.int_err = 0
        self.prev_int_err = 0

def pid(gains,err_struct,is_saturated=True,sat_bound=100):
    if is_saturated:
        sig_out = gains.Kp*err_struct.err + gains.Ki*err_struct.int_err + gains.Kd*err_struct.deriv_err
        if sig_out > sat_bound:
            sig_out = sat_bound
        elif sig_out < -sat_bound:
            sig_out = -sat_bound
    else:
        sig_out = gains.Kp*err_struct.err + gains.Ki*err_struct.int_err + gains.Kd*err_struct.deriv_err
    return sig_out

# def publish_nodes(pos_node, pub, pub_err, pub_rec_pose, r, move_cmd, err_ang, err_msg):

def have_same_sign(num1,num2):
    return (num1>=0 and num2>=0) or (num1<0 and num2<0)

def is_robot_stopped(twist_msg): 
    xdot = twist_msg.linear.x
    ydot = twist_msg.linear.y
    omegadot = twist_msg.angular.z
    tol = 0.001
    ang_tol = 0.01
    if abs(xdot) < tol and abs(ydot) < tol and abs(omegadot) < ang_tol:
        return True
    else:
        return False

def arg_parse():
    args = {}
    args['check_ref'] = rospy.get_param('~check_ref',True)
    args['enable_gain_topics'] = rospy.get_param('~enable_gain_topics',True)
    return args

def main():
    global check_ref
    rospy.init_node('PID_Controller')
    # test_mode specifies if gains are hard coded or received from topic
    args = arg_parse()
    test_mode = args['enable_gain_topics']
    check_ref = args['check_ref'] #TODO: implement this
    # setup publisher
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    ang_pub_err = rospy.Publisher('/ang_err', err_vals, queue_size=5)
    pos_pub_err = rospy.Publisher('/pos_err', err_vals,queue_size=5)
    pub_rec_pose = rospy.Publisher('/rec_pose', cur_pose, queue_size=5)
    pub_model_pose = rospy.Publisher('/model_pose', ModelStates, queue_size=5)
    r = rospy.Rate(40)
    
    # setup subscriptions
    pos_node = SubscriberNode_Repeat(topic='/gazebo/model_states',msg = ModelStates,msg_object = ModelStates(),repeat_topic_node = pub_model_pose)
    ref_node = SubscriberNode(topic='/reference_pose',msg = Reference_Pose,msg_object = Reference_Pose())
    
    dbg = True
    debug_interval = 20
    
    # ang_gains = PID_gains(Kp=1,Ki=0.001,Kd=0)
    # pos_gains = PID_gains(Kp=0.15,Ki=0.003,Kd=0)
    ang_gains = PID_gains(Kp=1,Ki=0.001,Kd=0)
    pos_gains = PID_gains(Kp=0.17,Ki=0.003,Kd=0.01)
    pos_err = err_struct(err_max=50)
    ang_err = err_struct(err_max=15)
    
    if test_mode:
        pos_gains_node=SubscriberNodeUpdateGains(topic='/pos_gains',msg=PID_Gains,msg_object=PID_Gains(),gains_obj=pos_gains)
        ang_gains_node=SubscriberNodeUpdateGains(topic='/ang_gains',msg=PID_Gains,msg_object=PID_Gains(),gains_obj=ang_gains)

    iterations=0 # used for debugging

    first_time = True
    controller_active = False
    while not ref_node.is_received() and not rospy.is_shutdown():
        if first_time:
            rospy.loginfo('waiting for ref msg')
            first_time = False

    while not rospy.is_shutdown():
        if ref_node.data.mode == 2:
            # is_body_angle = util.check_body_angle(format_model_state(pos_node),format_target(ref_node))
            # if not is_body_angle:
            activate_controller(pos_node, ref_node, pub, pos_pub_err, ang_pub_err, pub_rec_pose,pub_model_pose, r, dbg, debug_interval, pos_gains, ang_gains, pos_err, ang_err, iterations)
        else:
            is_at_ref = util.check_if_arrived(format_model_state(pos_node),format_target(ref_node))
            is_at_final_angle = is_final_angle(format_model_state(pos_node),ref_node.data.theta)
            if not is_at_ref or not is_at_final_angle:
                if not controller_active:
                    rospy.loginfo('activating controller: is_arrived = %s is_final_angle = %s' % (is_at_ref,is_at_final_angle)) 
                    controller_active = True
                activate_controller(pos_node, ref_node, pub, pos_pub_err, ang_pub_err, pub_rec_pose,pub_model_pose, r, dbg, debug_interval, pos_gains, ang_gains, pos_err, ang_err, iterations)
            else:
                if controller_active:
                    rospy.loginfo('turning off controller')
                    controller_active = False 

        # publish_nodes(pos_node, pub, pub_err, pub_rec_pose, r, move_cmd, err_ang, err_msg)

## Controller Functions
def activate_controller(pos_node, ref_node, pub, pos_pub_err, ang_pub_err, pub_rec_pose, pub_model_pose, r, dbg, debug_interval, pos_gains, ang_gains, pos_err, ang_err, iterations):
    global check_ref
    is_final_pose = False #determines if motion is complete
    while not is_final_pose and not rospy.is_shutdown():
        is_arrived = util.check_if_arrived(format_model_state(pos_node),format_target(ref_node))
        is_ref_angle = is_final_angle(format_model_state(pos_node),ref_node.data.theta)
        # if check_ref:
        completion_condition = not is_arrived or not is_ref_angle
        # else:
        #     completion_condition = not is_arrived
        if completion_condition:
            if ref_node.data.mode == 0:
                # first turn to face target, then move to target, then adjust to final reference angle
                rospy.loginfo('Using Mode 0')
                turn_to_target(pos_node, ref_node, pub, ang_pub_err, pub_rec_pose,pub_model_pose, r, dbg, debug_interval, ang_gains, ang_err, iterations)
                move_and_turn_to_target(pos_node, ref_node, pub, pos_pub_err, ang_pub_err, pub_rec_pose,pub_model_pose, r, dbg, debug_interval, ang_gains, ang_err, pos_gains, pos_err, iterations)
                # if check_ref:
                turn_to_ref_theta(pos_node, ref_node, pub, ang_pub_err, pos_pub_err, pub_rec_pose,pub_model_pose, r, dbg, debug_interval, ang_gains, ang_err, pos_gains, pos_err, iterations)
                is_final_pose = True
                iterations = 0

            elif ref_node.data.mode == 1:
                # turn and move to target at the same time
                rospy.loginfo('Using Mode 1')
                move_and_turn_to_target(pos_node, ref_node, pub, pos_pub_err, ang_pub_err, pub_rec_pose,pub_model_pose, r, dbg, debug_interval, ang_gains, ang_err, pos_gains, pos_err, iterations)
                # if check_ref:
                turn_to_ref_theta(pos_node, ref_node, pub, ang_pub_err, pos_pub_err, pub_rec_pose,pub_model_pose, r, dbg, debug_interval, ang_gains, ang_err, pos_gains, pos_err, iterations)
                is_final_pose = True
                iterations = 0

            elif ref_node.data.mode == 2:
                #this is for testing individual functions
                rospy.loginfo('using mode 2')
                turn_to_target(pos_node, ref_node, pub, ang_pub_err, pub_rec_pose,pub_model_pose, r, dbg, debug_interval, ang_gains, ang_err, iterations)

    rospy.loginfo('Reached Final Pose')
    move_cmd = Twist()
    pub.publish(move_cmd)

def turn_to_target(pos_node, ref_node, pub, pub_err, pub_rec_pose,pub_model_pose, r, dbg, debug_interval, ang_gains, ang_err, iterations):
    rospy.loginfo('Turning To Target')
    stop_time_started = False
    move_complete = False
    while not move_complete and not rospy.is_shutdown():
        while not util.check_body_angle(format_model_state(pos_node),format_target(ref_node)) and not rospy.is_shutdown():
            # turn to face target
            move_cmd = Twist() #reinitalize move_cmd
            
            _,err_ang = util.calc_error(format_model_state(pos_node),format_target(ref_node))
            ang_err.record_err(err_ang)
            err_msg = ang_err.make_err_val_msg(ang_gains)
            move_cmd.angular.z = pid(ang_gains,ang_err,sat_bound=ANG_SAT)
            
            pub_rec_pose.publish(make_rec_pose_msg(pos_node,err_ang))
            pub_err.publish(err_msg)
            pub.publish(move_cmd)
            r.sleep()
            rospy.sleep(CTRL_RATE)
        
            iterations += 1
          
        if not ref_node.data.mode == 2:
            move_complete=True


def move_and_turn_to_target(pos_node, ref_node, pub, pos_err_node, ang_err_node, pub_rec_pose,pub_model_pose, r, dbg, debug_interval, ang_gains, ang_err, pos_gains, pos_err, iterations):
    global check_ref
    # move and turn to target
    rospy.loginfo('Moving And Turning To Target')
    while not util.check_if_arrived(format_model_state(pos_node),format_target(ref_node)) and not rospy.is_shutdown():
        
        move_cmd = Twist()
        err_pos, err_ang = util.calc_error(format_model_state(pos_node),format_target(ref_node))
        ang_err.record_err(err_ang)
        pos_err.record_err(err_pos) 
        pos_err_msg = pos_err.make_err_val_msg(pos_gains)
        ang_err_msg = ang_err.make_err_val_msg(ang_gains)
        move_cmd.angular.z = pid(ang_gains,ang_err,sat_bound=ANG_SAT)
        move_cmd.linear.x = pid(pos_gains, pos_err,sat_bound=POS_SAT)
        
        pub_rec_pose.publish(make_rec_pose_msg(pos_node,err_ang))
        pos_err_node.publish(pos_err_msg)
        ang_err_node.publish(ang_err_msg)
        pub.publish(move_cmd)
        iterations += 1
        r.sleep()
        rospy.sleep(CTRL_RATE)
    pub.publish(Twist())
    is_first = True
    model = find_model_index(pos_node)
    if check_ref:
        while not is_robot_stopped(pos_node.data.twist[model]):
            if is_first:
                rospy.loginfo('waiting for robot to stop')
                is_first = False
            err_pos, err_ang = util.calc_error(format_model_state(pos_node),format_target(ref_node))
            ang_err.record_err(err_ang)
            pos_err.record_err(err_pos) 
            pos_err_msg = pos_err.make_err_val_msg(pos_gains)
            ang_err_msg = ang_err.make_err_val_msg(ang_gains)
            pub_rec_pose.publish(make_rec_pose_msg(pos_node,err_ang))
            pos_err_node.publish(pos_err_msg)
            ang_err_node.publish(ang_err_msg)
            r.sleep()

def turn_to_ref_theta(pos_node, ref_node, pub, ang_err_node, pos_err_node, pub_rec_pose,pub_model_pose, r, dbg, debug_interval, ang_gains, ang_err, pos_gains, pos_err, iterations):
    rospy.loginfo('Turning To Final Reference Pose')
    ang_err.reset_int_error()
    while not is_final_angle(format_model_state(pos_node),ref_node.data.theta) and not rospy.is_shutdown():
        #face final reference pose
        
        move_cmd = Twist()
        err_pos, _= util.calc_error(format_model_state(pos_node),format_target(ref_node))
        cur_state = format_model_state(pos_node)
        err_ang = util.calc_ang_error(cur_state['theta'],ref_node.data.theta)
        pos_err.record_err(err_pos) 
        ang_err.record_err(err_ang)
        pos_err_msg = pos_err.make_err_val_msg(pos_gains)
        ang_err_msg = ang_err.make_err_val_msg(ang_gains)
        move_cmd.angular.z = pid(ang_gains,ang_err,sat_bound=ANG_SAT)
        move_cmd.linear.x = pid(pos_gains, pos_err)

        pos_err_node.publish(pos_err_msg)
        ang_err_node.publish(ang_err_msg)
        pub.publish(move_cmd)
        iterations += 1
        r.sleep()
        rospy.sleep(CTRL_RATE)
    pub.publish(Twist())
    # r.sleep()
    # r.sleep(2)

def is_final_angle(cur_state,ref_theta):
    # checks if body is at final reference angle
    err_ang = util.calc_ang_error(cur_state['theta'],ref_theta)
    # debug_info('is_final_angle',ref_theta=ref_theta,cur_theta=cur_state['theta'])
    if abs(err_ang) < math.pi/180:
        result = True
        # rospy.loginfo('facing final angle')
    else:
        result = False
    return result

## Utility Functions
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
    
def make_rec_pose_msg(pos_node,ang_err):
    model = find_model_index(pos_node)
    msg_out = cur_pose()
    msg_out.x = pos_node.data.pose[model].position.x
    msg_out.y = pos_node.data.pose[model].position.y
    formatted_node = format_model_state(pos_node)
    msg_out.theta = formatted_node['theta']
    msg_out.dir_theta = ang_err + formatted_node['theta']
    msg_out.xdot = pos_node.data.twist[model].linear.x
    msg_out.ydot = pos_node.data.twist[model].linear.y
    msg_out.thetadot = pos_node.data.twist[model].angular.z
    return msg_out

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

def is_debug_iter(dbg_state,interval,iterations):
    if dbg_state and iterations % interval == 0:
        return True
    else:
        return False

def debug_info(info,**kwargs):
    output_string = info + " "
    for key,value in kwargs.iteritems():
        output_string += str(key) + ": " + str(value) + ','
    rospy.loginfo(output_string.rstrip(','))

if __name__ == "__main__":
    main()
