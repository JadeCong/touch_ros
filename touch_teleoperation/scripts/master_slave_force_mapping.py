#!/usr/bin/env python

import rospy
from dynamic_reconfigure.server import Server
from touch_teleoperation.cfg import force_scale_paramConfig
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float64
from scipy.spatial.transform import Rotation as R
import numpy as np


# define the feedback_force_index and feedback_force_scale
feedback_force_index, feedback_force_scale = [], []


def dynamic_reconfigure_force_scale_param_callback(config, level):
    # declare the global variables
    global feedback_force_scale
    
    # set the feedback_force_scale
    feedback_force_scale = [config.absolute_force_scale_x, config.absolute_force_scale_y, config.absolute_force_scale_z, 
                            config.absolute_force_scale_rx, config.absolute_force_scale_ry, config.absolute_force_scale_rz]
    
    return config

def force_mapping_callback(msg, args):
    # get the feedback_force msgs
    temp_feedback_force = msg
    args[4] = feedback_force_scale
    
    # feedback force mapping according to the force_mapping_order and absolute_force_scale of slave robot relative to master robot
    # set the header of feedback_force msgs
    args[1].header.stamp = rospy.Time.now()
    args[1].header.frame_id = args[2]
    # mapping the coordinate and scale of feedback_force msgs
    args[1].wrench.force.x = args[4][0] * eval("temp_feedback_force.wrench.force." + args[3][0])  # x
    args[1].wrench.force.y = args[4][1] * eval("temp_feedback_force.wrench.force." + args[3][1])  # y
    args[1].wrench.force.z = args[4][2] * eval("temp_feedback_force.wrench.force." + args[3][2])  # z
    args[1].wrench.torque.x = args[4][3] * eval("temp_feedback_force.wrench.torque." + args[3][3])  # rx
    args[1].wrench.torque.y = args[4][4] * eval("temp_feedback_force.wrench.torque." + args[3][4])  # ry
    args[1].wrench.torque.z = args[4][5] * eval("temp_feedback_force.wrench.torque." + args[3][5])  # rz
    
    # publish the feedback_force msgs
    args[0].publish(args[1])

def master_slave_force_mapping_node():
    # declare the global variables
    global feedback_force_index, feedback_force_scale
    
    # init ros node
    rospy.init_node("master_slave_force_mapping", anonymous=True)
    
    # get ros parameters from parameter server
    slave_force_topic = rospy.get_param("/touch/master_slave_force_mapping/slave_force_topic")
    master_force_frame_id = rospy.get_param("/touch/master_slave_force_mapping/master_force_frame_id")
    force_mapping_order = rospy.get_param("/touch/master_slave_force_mapping/force_mapping_order")
    absolute_force_scale = rospy.get_param("/touch/master_slave_force_mapping/absolute_force_scale")
    
    # set the feedback_force_index and feedback_force_scale
    master_robot_coordinate_index = ['x','y','z','rx','ry','rz']
    for idx in range(len(force_mapping_order)):
        if 'r' not in force_mapping_order[idx]:
            feedback_force_index.append(force_mapping_order[idx])
        elif 'r' in force_mapping_order[idx]:
            feedback_force_index.append(force_mapping_order[idx].strip('r'))
    for item in master_robot_coordinate_index:
        feedback_force_scale.append(absolute_force_scale[force_mapping_order.index(item)])
    
    # start the dynamic reconfigure parameter server
    dynamic_reconfigure_parameter_server = Server(force_scale_paramConfig, dynamic_reconfigure_force_scale_param_callback)
    
    # define the publisher and msgs for master force feedback
    pub_feedback_force = rospy.Publisher("/touch/master_touch/feedback_force", WrenchStamped, queue_size=1)
    feedback_force = WrenchStamped()
    
    # define the subscriber for getting feedback force from slave hand
    sub_feedback_force = rospy.Subscriber(slave_force_topic, WrenchStamped, 
                                          callback=force_mapping_callback, 
                                          callback_args=[pub_feedback_force, feedback_force, master_force_frame_id, feedback_force_index, feedback_force_scale], 
                                          queue_size=1)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == "__main__":
    # run ros node
    master_slave_force_mapping_node()
