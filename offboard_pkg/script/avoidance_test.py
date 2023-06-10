#!/usr/bin/env python
#coding=utf-8

import rospy
import os
import json
import numpy as np
import math
import time
import threading
import Tkinter
from geometry_msgs.msg import TwistStamped, Quaternion, PoseStamped
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Empty
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import SetMavFrame
from mavros_msgs.msg import State, RCIn, HomePosition, PositionTarget
from mavros_msgs.msg import Thrust
from utils_obs import Utils
from Queue import Queue
#from rflysim_ros_pkg.msg import Obj

from math import atan2, pi
from random import random
from assemble_cmd import Px4Controller
from swarm_msgs.msg import BoundingBox, BoundingBoxes
from obs_avoidance import Avoidance


# Simulation of RealFlight
current_state = State()
ch5, ch6, ch7, ch8, ch9, ch11, ch14 = 0, 0, 0, 0, 1, 1, 1
is_initialize_mav, is_initialize_vel, is_initialize_rc, is_initialize_img = False, False, False, False

ch20 = 0
mav_pos = [0, 0, 0]
mav_original_angle = [0, 0, 0]
# mav_vel = [0, 0, 0]
mav_vel = np.array([0, 0, 0])
mav_yaw = 0
mav_R = np.zeros((3,3))
mav_id = 1
Initial_pos = [0, 0, 0]
pos_i = [0, 0, 0, 0, 0]
pos_i_raw = [0, 0, 0, 0, 0]
pos_i_ekf = [0, 0, 0, 0, 0]
image_failed_cnt = 0
state_name = "InitializeState"

#oldhover
idle_command = TwistStamped()

#attack
command = PositionTarget()
command.coordinate_frame = PositionTarget.FRAME_LOCAL_NED 
command.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ \
                  + PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                  + PositionTarget.IGNORE_YAW

#rotate
rotate_rat = pi / 4
rotate_command = PositionTarget()
rotate_command.coordinate_frame = PositionTarget.FRAME_LOCAL_NED 
rotate_command.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ \
                         + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                         + PositionTarget.IGNORE_YAW
rotate_command.yaw_rate = rotate_rat
rotate_command.velocity.x, rotate_command.velocity.y, rotate_command.velocity.z = 0, 0, 0

#hover
hover_command = PositionTarget()
hover_command.coordinate_frame = PositionTarget.FRAME_LOCAL_NED 
hover_command.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ \
                        + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                        + PositionTarget.IGNORE_YAW_RATE
hover_command.yaw = 0
hover_command.velocity.x, rotate_command.velocity.y, rotate_command.velocity.z = 0, 0, 0


q = Queue()
maxQ = 100
sumQ = 0.0
home_dx, home_dy = 0, 0
depth = -1
original_offset = np.array([0, 0, 0])


impact_distance = 0.6
arrive_distance = 1
left_distance = 2
attack_start_distance = 20
highspeed_distance = 20
middlespeed_distance = 10
offset_distance = 5
high_speed = 5
middle_speed = 3
slow_speed = 1


sphere_pos_1 = np.array([-30, 80, 10])
sphere_pos_2 = np.array([0, 100, 10])
sphere_pos_3 = np.array([25, 120, 10])
sphere_all_pos = [sphere_pos_1, sphere_pos_2, sphere_pos_3]
sphere_true_pos_1 = sphere_pos_1 + offset_distance *(2 * np.array([random(), random(), 0.3 * random()]) - np.array([1, 1, 0.3]))
sphere_true_pos_2 = sphere_pos_2 + offset_distance *(2 * np.array([random(), random(), 0.3 * random()]) - np.array([1, 1, 0.3]))
sphere_true_pos_3 = sphere_pos_3 + offset_distance *(2 * np.array([random(), random(), 0.3 * random()]) - np.array([1, 1, 0.3]))
sphere_true_all_pos = [sphere_true_pos_1, sphere_true_pos_2, sphere_true_pos_3]
sphere_all_id = [100, 101, 102]
# sphere_vel = np.array([-5, 0, 2])
sphere_vel = np.array([0, 0, 0])
sphere_acc = np.array([0, 0, -0.5])
# sphere_vel = np.array([-5, 0, 0])
# sphere_acc = np.array([0, 0, 0])

sphere_feb_pos = PoseStamped()
# obj_state = ModelState()

target_num = 0
sphere_pos = sphere_all_pos[target_num]


def spin():
    rospy.spin()

def state_cb(msg):
    global current_state
    current_state = msg

def mav_pose_cb(msg):
    global mav_pos, mav_yaw, mav_R, is_initialize_mav, mav_pitch, mav_roll
    is_initialize_mav = True
    mav_pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
    q0, q1, q2, q3 = msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z
    mav_yaw = math.atan2(2*(q0*q3 + q1*q2), 1-2*(q2*q2 + q3*q3))
    mav_pitch = math.asin(2*(q0*q2 - q1*q3))
    mav_roll = math.atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1 + q2*q2))
    R_ae = np.array([[q0**2+q1**2-q2**2-q3**2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2)],
                      [2*(q1*q2+q0*q3), q0**2-q1**2+q2**2-q3**2, 2*(q2*q3-q0*q1)],
                      [2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0**2-q1**2-q2**2+q3**2]])
    # R_ba = np.array([[0,1,0], [-1,0,0], [0,0,1]]) #mavros_coordinate to body_coordinate
    R_ba = np.array([[0,1,0], [-1,0,0], [0,0,1]]) #mavros_coordinate to baselink_coordinate  // body to enu  # body: right-front-up3rpos_est_body)
    mav_R = R_ae.dot(R_ba)
    # mav_R = R_ae

def mav_vel_cb(msg):
    global mav_vel, is_initialize_vel
    is_initialize_vel = True
    # mav_vel = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]
    mav_vel = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])

def rcin_cb(msg):
    global ch5, ch6, ch7, ch8, ch9, ch11, ch14, is_initialize_rc
    is_initialize_rc = True
    last_ch5, last_ch6, last_ch7, last_ch8, last_ch9, last_ch11, last_ch14 = ch5, ch6, ch7, ch8, ch9, ch11, ch14
    chs = msg.channels
    ch5 = 2 if chs[4] < 1300 else 1 if chs[4] < 1700 else 0
    ch6 = 2 if chs[5] < 1300 else 1 if chs[5] < 1700 else 0
    ch7 = 0 if chs[6] < 1300 else 1 if chs[6] < 1700 else 2
    ch8 = 0 if chs[7] < 1300 else 1 if chs[7] < 1700 else 2
    ch9 = 2 if chs[8] < 1300 else 1 if chs[8] < 1700 else 0
    ch11 = 1 if chs[10] < 1500 else 0
    ch14 = 1 if chs[10] < 1500 else 0
    if ch5!=last_ch5 or ch6!=last_ch6 or ch7!=last_ch7 or ch8!=last_ch8 or ch9!=last_ch9 or ch11!=last_ch11 or ch14!=last_ch14:
        print("ch5: {}, ch6: {}, ch7: {}, ch8: {}, ch9: {}, ch11: {}, ch14: {}".format(ch5, ch6, ch7, ch8, ch9, ch11, ch14))

def call(event):
    global ch5, ch6, ch7, ch8, ch9, ch20
    k = event.keysym
    if k == "m":
        ch6 = 0
    elif k == "h":
        ch20 = 1
    elif k == "o":
        ch8 = 1
    elif k == "p":
        ch8 = 0
    elif k == "c":
        ch9 = (ch9 + 1) % 2
    elif k == "a":
        ch7 = 1
    elif k == "b":
        ch7 = 0
    time.sleep(0.02)

def read_kbd_input():
    global is_initialize_rc
    is_initialize_rc = True
    win = Tkinter.Tk()
    frame = Tkinter.Frame(win,width=100,height=60)
    frame.bind("<Key>",call)
    frame.focus_set()
    frame.pack()
    win.mainloop()


def minAngleDiff(a, b):
    diff = a - b
    if diff < 0:
        diff += 2*np.pi
    if diff < np.pi:
        return diff
    else:
        return diff - 2*np.pi

def angleLimiting(a):
    if a > np.pi:
        return a - 2*np.pi
    if a < -np.pi:
        return a + 2*np.pi
    return a











if __name__=="__main__":


    rospy.init_node('offb_node', anonymous=True)
    
    oa = Avoidance()

    rate = rospy.Rate(50)#50
    
    last_request = rospy.Time.now()

    # start
    cnt = -1

    while not rospy.is_shutdown():
        cnt += 1
        #print('distance:',oa.obs_distance)
        if oa.obs_distance < oa.obs_distance_min:
            oa.local_obs_avoidance()
            rate.sleep()
            continue
        else:
            oa.moveByVelocityYawrateENU(0., 0., 0., 0.)
            rate.sleep()
