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

from math import atan2, pi
from random import random
from assemble_cmd import Px4Controller
from swarm_msgs.msg import BoundingBox, BoundingBoxes


# Simulation of RealFlight
current_state = State()
ch5, ch6, ch7, ch8, ch9, ch11, ch14 = 0, 0, 0, 0, 1, 1, 1
is_initialize_mav, is_initialize_vel, is_initialize_rc, is_initialize_img = False, False, False, False

ch20 = 0
mav_pos = [0., 0., 0.]
mav_original_angle = [0., 0., 0.]
# mav_vel = [0, 0, 0]
mav_vel = np.array([0., 0., 0.])
mav_yaw = 0
mav_R = np.zeros((3,3))
mav_id = 1
Initial_pos = [0., 0., 0.]
pos_i = [0., 0., 0., 0., 0.]
pos_i_raw = [0., 0., 0., 0., 0.]
pos_i_ekf = [0., 0., 0., 0., 0.]
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
                        + PositionTarget.IGNORE_YAW
hover_command.yaw_rate = 0
hover_command.velocity.x, rotate_command.velocity.y, rotate_command.velocity.z = 0, 0, 0


q = Queue()
maxQ = 100
sumQ = 0.0
home_dx, home_dy = 0, 0
depth = -1
original_offset = np.array([0., 0., 0.])


picminsize = 30.0
impact_distance = 0.6
arrive_distance = 1.
left_distance = 2.
attack_start_distance = 20.
highspeed_distance = 20.
middlespeed_distance = 10.
offset_distance = 5.
high_speed = 5.
middle_speed = 3.
slow_speed = 1.
sphere_speed = 3.
image_failed_max_cnt = 30

#real
#drone position (need to change)
sphere_pos_1 = np.array([0., -12., 12.])
sphere_pos_2 = np.array([15., -58., 12.])
#drone yaw
drone_all_yaw = [- pi / 2, pi / 2]
sphere_all_pos = [sphere_pos_1, sphere_pos_2]
sphere_all_id = [100, 101]


#sim
sphere_true_pos_1 = np.array([10., 45., 10.])
sphere_true_pos_2 = np.array([30., 145., 10.])
sphere_true_pos_3 = np.array([10., 45., 10.])
sphere_true_pos_4 = np.array([30., 145., 10.])
sphere_true_all_pos = [sphere_true_pos_1, sphere_true_pos_2]

sphere_disappear_flag = [0, 0]


# sphere_pos_3 = np.array([25, 120, 10])
# sphere_all_pos = [sphere_pos_1, sphere_pos_2, sphere_pos_3]
# sphere_true_pos_1 = sphere_pos_1 + offset_distance *(2 * np.array([random(), random(), 0.3 * random()]) - np.array([1, 1, 0.3]))
# sphere_true_pos_2 = sphere_pos_2 + offset_distance *(2 * np.array([random(), random(), 0.3 * random()]) - np.array([1, 1, 0.3]))
# sphere_true_pos_3 = sphere_pos_3 + offset_distance *(2 * np.array([random(), random(), 0.3 * random()]) - np.array([1, 1, 0.3]))
# sphere_true_all_pos = [sphere_true_pos_1, sphere_true_pos_2, sphere_true_pos_3]
# sphere_all_id = [100, 101, 102]
# sphere_vel = np.array([-5, 0, 2])
sphere_vel = np.array([0., 0., 0.])
sphere_acc = np.array([0., 0., -0.5])
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

def pos_image_cb(msg):
    global is_initialize_img, pos_i_raw, pos_i, image_failed_cnt, mav_id
    is_initialize_img = True
    # print("msg_data: {}".format(msg.data))
    sphere_num = len(msg.bounding_boxes)
    if sphere_num > 0:
        image_failed_cnt = 0
    else:
        image_failed_cnt += 1
    if image_failed_cnt <= image_failed_max_cnt and image_failed_cnt > 0:
        pass
    else:
        if sphere_num > 0:
            impact_num = 0
            xmiddle = (msg.bounding_boxes[impact_num].xmin + msg.bounding_boxes[impact_num].xmax) / 2
            ymiddle = (msg.bounding_boxes[impact_num].ymin + msg.bounding_boxes[impact_num].ymax) / 2
            picwidth = msg.bounding_boxes[impact_num].xmax - msg.bounding_boxes[impact_num].xmin
            picheight = msg.bounding_boxes[impact_num].ymax - msg.bounding_boxes[impact_num].ymin
            if picwidth * picheight > picminsize:
                outdata = [xmiddle, ymiddle, picwidth, picheight]
            else:
                outdata = [-1, -1, -1, -1]
            pos_i_raw = outdata
            pos_i = pos_i_raw
        else:
            outdata = [-1, -1, -1, -1]
            pos_i_raw = outdata
            pos_i = pos_i_raw
    
    # if xmiddle <= 0:
    #     image_failed_cnt += 1
    # else:
    #     image_failed_cnt = 0
    # if image_failed_cnt <= 20 and image_failed_cnt > 0:
    #     pass
    # else:
    #     ymiddle = (msg.bounding_boxes[0].ymin + msg.bounding_boxes[0].ymax) / 2
    #     picwidth = msg.bounding_boxes[0].xmax - msg.bounding_boxes[0].xmin
    #     picheight = msg.bounding_boxes[0].ymax - msg.bounding_boxes[0].ymin
    #     outdata = [xmiddle, ymiddle, picwidth, picheight]
    #     pos_i_raw = outdata
    #     pos_i = pos_i_raw
    # print("pos_i_raw: {}".format(pos_i_raw))

def pos_image_ekf_cb(msg):
    global pos_i_ekf, pos_i_raw, pos_i

    # xmiddle = (msg.bounding_boxes[0].xmin + msg.bounding_boxes[0].xmax) / 2
    # ymiddle = (msg.bounding_boxes[0].ymin + msg.bounding_boxes[0].ymax) / 2
    # picwidth = msg.bounding_boxes[0].xmax - msg.bounding_boxes[0].xmin
    # picheight = msg.bounding_boxes[0].ymax - msg.bounding_boxes[0].ymin
    # outdata = [xmiddle, ymiddle, picwidth, picheight]
    # pos_i_ekf = outdata
    pos_i_ekf = msg_data
    # If we don't consider the safety of the aircraft when the target is lost, use pos_i_ekf when pos_i_raw[0]<0.
    if abs(pos_i_ekf[0] - pos_i_raw[0]) < 10 and abs(pos_i_ekf[1] - pos_i_raw[1]) < 10:
        pos_i = pos_i_ekf
    else:
        pos_i = pos_i_raw
    # print("pos_i_ekf: {}".format(pos_i_ekf))
    # print("pos_i: {}".format(pos_i))

sphere_road_num = [1, 3]

def sphere_control(time, sphere_id, sphere_type, is_move=False):

    global sphere_true_all_pos, sphere_all_id, sphere_road_num
    obj_msg = Obj()
    
    obj_msg.id = sphere_id
    sphere_num = sphere_all_id.index(sphere_id)

    # obj_msg.type = 152
    obj_msg.type = sphere_type
    newpos = sphere_true_all_pos[sphere_num]
    sphere_road = sphere_road_num[sphere_num]

    if sphere_road == 2:
        newpos[1] = 145
        newpos[0] = newpos[0] + sphere_speed * 0.02
        if newpos[0] > 30:
            newpos[0] = 30
            sphere_road_num[sphere_num] = 3
    elif sphere_road == 3:
        newpos[1] = newpos[1] - sphere_speed * 0.02
        newpos[0] = 30
        if newpos[1] < 45:
            newpos[1] = 45
            sphere_road_num[sphere_num] = 4
    elif sphere_road == 4:
        newpos[1] = 45
        newpos[0] = newpos[0] - sphere_speed * 0.02
        if newpos[0] < 10:
            newpos[0] = 10
            sphere_road_num[sphere_num] = 1
    else:
        newpos[1] = newpos[1] + sphere_speed * 0.02
        newpos[0] = 10
        if newpos[1] > 145:
            newpos[1] = 145
            sphere_road_num[sphere_num] = 2


    sphere_true_all_pos[sphere_num] = newpos

    obj_msg.position.x = newpos[0]
    obj_msg.position.y = newpos[1]
    obj_msg.position.z = newpos[2]
    obj_msg.size.x = 0.05
    obj_msg.size.y = 0.05
    obj_msg.size.z = 0.05

    sphere_pub.publish(obj_msg)


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


def sphere_set():
    global sphere_all_id, mav_id
    for i in range(len(sphere_all_id)):
        if sphere_disappear_flag[i] == 0:
            sphere_control(0, sphere_all_id[i], 152, ch8==1)
        else:
            sphere_control(0, sphere_all_id[i], 102, ch8==1)


def sphere_impact():
    global sphere_true_all_pos, sphere_all_id, sphere_pos, mav_pos, impact_distance
    if len(sphere_all_id) > 0:
        for i in range(len(sphere_all_id)):
            # print(len(sphere_all_id))
            diff_distance = np.linalg.norm(sphere_true_all_pos[i] - mav_pos)
            if diff_distance < impact_distance:
                sphere_disappear_flag[i] = 1









if __name__=="__main__":
    setting_file = open(os.path.join(os.path.expanduser('~'),"Rfly_Attack/src","settings.json"))
    setting = json.load(setting_file)
    # print(json.dumps(setting, indent=4))

    MODE = setting["MODE"]
    car_velocity = setting["car_velocity"]
    # follow_mode: 0, ll=follow_distance; 1, ll=norm(car_home, mav_home)
    follow_mode = setting["follow_mode"]
    follow_distance = setting["follow_distance"]
    FLIGHT_H = setting["FLIGHT_H"]
    if MODE == "RealFlight":
        u = Utils(setting["Utils"])
    elif MODE == "Simulation":
        u = Utils(setting["Simulation"])


    rospy.init_node('offb_node', anonymous=True)
    mav_id = rospy.get_param("~mav_id")
    print("mav_id:", mav_id)
    if mav_id == 1:
        sphere_all_pos = [sphere_pos_1, sphere_pos_2]
        drone_yaw = drone_all_yaw[0]
    else:
        sphere_all_pos = [sphere_pos_2, sphere_pos_1]
        drone_yaw = drone_all_yaw[1]
    sphere_pos = sphere_all_pos[target_num]


    px = Px4Controller()
    
    spin_thread = threading.Thread(target = spin)
    spin_thread.start()

    rospy.Subscriber("mavros/state", State, state_cb)
    rospy.Subscriber("mavros/local_position/pose", PoseStamped, mav_pose_cb)
    rospy.Subscriber("mavros/local_position/velocity_local", TwistStamped, mav_vel_cb)
    #HIL使用遥控器进行控制
    IsRC = setting["IsRC"]
    if MODE == "RealFlight":
        rospy.Subscriber("mavros/rc/in", RCIn, rcin_cb)
        image_center = [setting["Utils"]["WIDTH"] / 2.0, setting["Utils"]["HEIGHT"] / 2.0]
    elif MODE == "Simulation":
        from rflysim_ros_pkg.msg import Obj
        sphere_pub = rospy.Publisher("ue4_ros/obj", Obj, queue_size=10)
        image_center = [setting["Simulation"]["WIDTH"] / 2.0, setting["Simulation"]["HEIGHT"] / 2.0]

        if IsRC == True:
            rospy.Subscriber("mavros/rc/in", RCIn, rcin_cb)
        else:
            inputThread = threading.Thread(target=read_kbd_input)
            inputThread.start()
    else:
        raise Exception("Invalid MODE!", MODE)
    

    #rospy.Subscriber("tracker/pos_image", Float32MultiArray, pos_image_cb)
    rospy.Subscriber("tracker/pos_image", BoundingBoxes, pos_image_cb)
    rospy.Subscriber("tracker/pos_image_ekf", Float32MultiArray, pos_image_ekf_cb)
    local_vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    local_acc_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=10)
    # print("Publisher and Subscriber Created")

    # rospy.wait_for_service("mavros/setpoint_velocity/mav_frame")
    # frame_client = rospy.ServiceProxy('mavros/setpoint_velocity/mav_frame', SetMavFrame)
    # resp_frame = frame_client(8)
    # if resp_frame.success:
    #     # print("Set earth_FLU success!")
    # else:
    #     # print("Set frame failed!")

    rospy.wait_for_service("mavros/cmd/arming")
    arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
    rospy.wait_for_service("mavros/set_mode")
    set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
    # print("Clients Created")
    rate = rospy.Rate(50)#50
    
    # ensure the connection 
    while(not current_state.connected):
        # print("connected: {}".format(current_state.connected))
        rate.sleep()

    for i in range(100):
        local_vel_pub.publish(idle_command)
        rate.sleep()
        
    # switch into offboard
    # print("Creating Objects for services")
    offb_set_mode = SetMode()
    offb_set_mode.custom_mode = "OFFBOARD"
    arm_cmd = CommandBool()
    arm_cmd.value = True

    last_request = rospy.Time.now()

    # start
    cnt = -1
    controller_reset = True


    if MODE == "Simulation":
        sphere_set()


    while not rospy.is_shutdown():
        # print("time: {}".format(rospy.Time.now().to_sec() - last_request.to_sec()))
        cnt += 1
        if MODE == "Simulation":
            sphere_set()    
            sphere_impact()

        # if ch8 == 0:
        #     if current_state.mode == "OFFBOARD":
        #         resp1 = set_mode_client(0, "POSCTL")	# (uint8 base_mode, string custom_mode)
        #     if cnt % 10 == 0:
        #         print("Enter MANUAL mode")
        #     Initial_pos = mav_pos
        #     rate.sleep()
        #     continue
        # else:
        #     if current_state.mode != "OFFBOARD":
        #         resp1 = set_mode_client( 0,offb_set_mode.custom_mode )
        #         if resp1.mode_sent:
        #             print("Offboard enabled")
        #         last_request = rospy.Time.now()
        
        pos_info = {"mav_pos": mav_pos, "mav_vel": mav_vel, "mav_R": mav_R, "R_bc": np.array([[0,0,1], [1,0,0], [0,1,0]]), 
                    "mav_original_angle": mav_original_angle, "Initial_pos": Initial_pos}

        dlt_pos = sphere_pos - np.array(mav_pos)
        # print("dlt_pos: {}".format(dlt_pos))
        # print("mav_pos: {}".format(mav_pos))
        
        # cmd = u.DockingControllerFusion(pos_info, pos_i_raw)
        # cmd = u.BasicAttackController(pos_info, pos_i_raw, image_center)

        if ch7 >= 1:
            # cmd = u.RotateAttackController(pos_info, pos_i, image_center, controller_reset)
            cmd = u.RotateAttackAccelerationController2(pos_info, pos_i, controller_reset, drone_yaw)
            controller_reset = False
            target_distance = np.linalg.norm(sphere_pos - mav_pos)
            # 识别到图像才进行角速度控制
            if pos_i[1] > 0: 
                command.acceleration_or_force.x = cmd[0]
                command.acceleration_or_force.y = cmd[1]
                command.acceleration_or_force.z = cmd[2]
                command.yaw_rate = cmd[3]
                # print("cmd: {}".format(cmd))
                local_acc_pub.publish(command)
            # # 否则
            else:
                target_yaw = atan2(sphere_pos[1] - mav_pos[1], sphere_pos[0] - mav_pos[0])  
                if target_distance < arrive_distance:
                    local_acc_pub.publish(hover_command)
                else:
                    if target_distance > highspeed_distance:
                        mav_speed = high_speed
                    elif target_distance <= highspeed_distance and target_distance > middlespeed_distance:
                        mav_speed = middle_speed
                    else:
                        mav_speed = slow_speed
                    
                    px.moveToPositionOnceAsync(sphere_pos[0], sphere_pos[1], sphere_pos[2], drone_yaw, mav_speed)
                    
                    

        else:
            Initial_pos = mav_pos
            controller_reset = True
            #local_vel_pub.publish(idle_command)
            local_acc_pub.publish(hover_command)
        obs_pos = sphere_pos
        rate.sleep()
    rospy.spin()