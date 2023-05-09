#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from TurtleBot.msg import Reference_Pose
import swim_to_goal as util
import click

class SubscriberNode(object):
    def __init__(self,topic,msg):
        self.data = None
        self.prev_data = None
        # self.last_callback_time = 0
        # self.time_since_last_callback = None
        rospy.Subscriber(topic, msg, self.callback)
        # self.not_first_callback = False

    def callback(self, data):
        self.prev_data = self.data
        self.data = data
        # callback_time = rospy.get_time()
        # if self.not_first_callback:
        #     self.time_since_last_callback = callback_time - self.last_callback_time
        # self.last_callback_time = callback_time

class fakeSubscriberNode(object):
    '''used to match the interface needed for activating the controller'''
    def __init__(self,ref_x,ref_y,ref_theta,mode):
        msg = Reference_Pose()
        msg.x = ref_x
        msg.y = ref_y
        msg.theta = ref_theta
        msg.mode = mode
        self.data = msg
class PID_gains(object):
    def __init__(self,Kp,Ki,Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def update_gains(self,Kp,Ki,Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

class err_struct(object):
    def __init__(self):
        self.err = 0
        self.int_err = 0
        self.deriv_error = 0
        self.prev_err = 0
        self.prev_int_err = 0
        self.error_max = 100
    
    def record_err(self,new_err,dt):
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
        is_first = True
        total_err = self.prev_int_err + new_err
        if total_err > self.error_max:
            total_err = self.error_max
            if is_first:
                rospy.loginfo('Upper Error Max Hit')
                is_first = False
        elif total_err < self.error_max:
            total_err = -self.error_max
            if is_first:
                rospy.loginfo('Lower Error Max Hit')
                is_first = False
        else:
            is_first = True
        return total_err

    def update_d_error(self,new_err):
        self.deriv_error = (new_err - self.prev_err)

def get_gains_from_topic():
    #TODO implement getting PID gain values from topic
    return None

def pid(gains,err_struct):
    return gains.Kp*err_struct.err + gains.Ki*err_struct.int_err + gains.Kd*err_struct.d_err

def format_target(node):
    return (node.data.pose.x, node.data.pose.y)

def is_final_angle(pose,ref_theta):
    # checks if body is at final reference angle
    err_ang = ref_theta - pose.theta
    if abs(err_ang) < util.math.pi/180:
        result = True
    else:
        result = False
    return result

def debug_interval(dbg_state,iterations,interval):
    if dbg_state and iterations % interval == 0:
        return True
    else:
        return False

def debug_info(info,**kwargs):
    output_string = info + " "
    for key,value in kwargs.iteritems():
        output_string =+ str(key) + ": " + str(value) + ','
    rospy.loginfo(output_string[:-2])

def request_mode():
    return util.request_number('')

def request_ref_point():
    x = util.request_number('x',is_bounds=False)
    y = util.request_number('y',is_bounds=False)
    theta = util.request_number('theta (radians)',is_bounds=False)
    return (x,y,theta)

def main():
    rospy.init_node('PID_Controller')
    # setup subscriptions
    pos_node = SubscriberNode(topic='/turtle1/pose',msg = Pose)
    ref_node = SubscriberNode(topic='/reference_pose',msg = Reference_Pose)
    mp_node = SubscriberNode(topic='/')
    # setup publisher
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    r = rospy.Rate(10)
    dbg = True
    debug_interval = 10
    test_mode = True
   
    pos_gains = PID_gains(Kp=1.5,Ki=0,Kd=0)
    ang_gains = PID_gains(Kp=3,Ki=0,Kd=0)
    pos_err = err_struct()
    ang_err = err_struct()
    
    iterations = 0 # used for debugging

    if test_mode:
        # in this mode gains and references are comanded my motion planner node
        activate_controller(pos_node, ref_node, pub, r, dbg, debug_interval, pos_gains, ang_gains, pos_err, ang_err, iterations)
    else:
        # gains are hard coded and references are inputted by user
        mode = request_mode()
        ref_value = request_ref_point()
        fake_ref_node = fakeSubscriberNode(*ref_value,mode)
        activate_controller(pos_node, fake_ref_node, pub, r, dbg, debug_interval, pos_gains, ang_gains, pos_err, ang_err, iterations)


def activate_controller(pos_node, ref_node, pub, r, dbg, debug_interval, pos_gains, ang_gains, pos_err, ang_err, iterations):
    is_final_pose = False
    while not is_final_pose() and not rospy.is_shutdown():
        is_arrived = util.check_if_arrived(pos_node.data,format_target(ref_node))
        if not is_arrived:
            if ref_node.data.mode == 0:
                # first turn to face target, then move to target, then adjust to final reference angle
                rospy.loginfo('Using Mode 0')
                turn_to_target(pos_node, ref_node, pub, r, dbg, debug_interval, ang_gains, ang_err, iterations)
                move_to_target(pos_node, ref_node, pub, r, dbg, debug_interval, pos_gains, pos_err, iterations)
                turn_to_ref_theta(pos_node, ref_node, pub, r, dbg, debug_interval, ang_gains, ang_err, iterations)
                is_final_pose = True

            elif ref_node.data.mode == 1:
                # turn and move to target at the same time
                rospy.loginfo('Using Mode 1')
                move_and_turn_to_target(pos_node, ref_node, pub, r, dbg, debug_interval, ang_gains, ang_err, pos_gains, pos_err, iterations)
                is_final_pose = True

def move_and_turn_to_target(pos_node, ref_node, pub, r, dbg, debug_interval, ang_gains, ang_err, pos_gains, pos_err, iterations):
        # move and turn to target
        rospy.loginfo('Moving And Turning To Target')
        move_cmd = Twist()
        err_pos, err_ang = util.calc_error(pos_node.data,format_target(ref_node))
        ang_err.record_err(err_ang)
        pos_err.record_err(err_pos)
        move_cmd.angular.z = pid(ang_gains,ang_err)
        move_cmd.linear.x = pid(pos_gains, pos_err)
        
        if debug_interval(dbg,debug_interval,iterations):
            debug_info("Position:",err=pos_err.err,int_err=pos_err.int_err,d_error=pos_err.deriv_error,prev_err=pos_err.prev_error,prev_int_err=pos_err.prev_int_err)
            debug_info("Angular:",err=ang_err.err,int_err=ang_err.int_err,d_error=ang_err.deriv_error,prev_err=ang_err.prev_error,prev_int_err=ang_err.prev_int_err)
        
        pub.publish(move_cmd)
        iterations += 1
        r.sleep()

def turn_to_ref_theta(pos_node, ref_node, pub, r, dbg, debug_interval, ang_gains, ang_err, iterations):
    while not is_final_angle(pos_node.data.pose,ref_node.data.pose.theta) and not rospy.is_shutdown():
        #face final reference pose
        rospy.loginfo('Turning To Final Reference Pose')
        move_cmd = Twist()
        _, err_ang = util.calc_error(pos_node.data,format_target(ref_node))
        ang_err.record_err(err_ang)
        move_cmd.angular.z = pid(ang_gains,ang_err)

        if debug_interval(dbg,debug_interval,iterations):
            debug_info("Angular:",err=ang_err.err,int_err=ang_err.int_err,d_error=ang_err.deriv_error,prev_err=ang_err.prev_error,prev_int_err=ang_err.prev_int_err)
        
        pub.publish(move_cmd)
        iterations += 1
        r.sleep()

def turn_to_target(pos_node, ref_node, pub, r, dbg, debug_interval, ang_gains, ang_err, iterations):
    while not util.check_body_angle(pos_node.data,format_target(ref_node)) and not rospy.is_shutdown():
        # turn to face target
        rospy.loginfo('Turning To Target')
        move_cmd = Twist() #reinitalize move_cmd
        _,err_ang = util.calc_error(pos_node.data,format_target(ref_node))
        ang_err.record_err(err_ang)
        move_cmd.angular.z = pid(ang_gains,ang_err)

        if debug_interval(dbg,debug_interval,iterations):
            debug_info("Angular:",err=ang_err.err,int_err=ang_err.int_err,d_error=ang_err.deriv_error,prev_err=ang_err.prev_error,prev_int_err=ang_err.prev_int_err)
        
        pub.publish(move_cmd)
        iterations += 1
        r.sleep()

def move_to_target(pos_node, ref_node, pub, r, dbg, debug_interval, pos_gains, pos_err, iterations):
    while not util.check_if_arrived(pos_node.data,format_target(ref_node)) and not rospy.is_shutdown(): 
        # move to target
        rospy.loginfo('Moving To Target')
        move_cmd = Twist()
        err_pos,_ = util.calc_error(pos_node.data,format_target(ref_node))
        pos_err.record_err(err_pos)
        move_cmd.linear.x = pid(pos_gains, pos_err)

        if debug_interval(dbg,debug_interval,iterations):
            debug_info("Position:",err=pos_err.err,int_err=pos_err.int_err,d_error=pos_err.deriv_error,prev_err=pos_err.prev_error,prev_int_err=pos_err.prev_int_err)
        
        pub.publish(move_cmd)
        iterations += 1
        r.sleep()


if __name__ == "__main__":
    main()
