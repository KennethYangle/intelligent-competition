#!/usr/bin/env python
# coding=utf-8

import time
import numpy as np
import os
import json
import rospy, rospkg
from geometry_msgs.msg import TwistStamped
from swarm_msgs.msg import Action, FixedPoint, BoundingBoxes
from mavros_msgs.msg import State, PositionTarget, RCIn
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Bool
from sensor_msgs.msg import NavSatFix

# 无人机控制类
class Avoidance:
    def __init__(self, drone_id=1, level_id=1, is_flight=1, scene="32s"):
        self.arm_state = False
        self.offboard_state = False
        self.state = None
        self.command = TwistStamped()
        self.start_point = PoseStamped()
        self.start_point.pose.position.z = 4
        self.drone_id = drone_id
        self.level_id = level_id
        self.drone_name = "drone_{}".format(self.drone_id)
        self.scene = scene
        self.task_ready = False
        self.rate = rospy.Rate(50)

        self.is_initialize_pos, self.is_initialize_vel = False, False
        self.ch5, self.ch6, self.ch7, self.ch8, self.ch9, self.ch11, self.ch14 = 0, 0, 0, 0, 0, 0, 0
        self.mav_yaw = 0
        self.mav_pos = np.array([0., 0., 0.])
        self.mav_vel = np.array([0., 0., 0.])
        self.mav_R = np.identity(3)

        self.another_mav_yaw = 0
        self.another_mav_pos = np.array([0., 0., 0.])
        self.another_mav_vel = np.array([0., 0., 0.])
        self.another_mav_R = np.identity(3)
        self.obs_distance = 100.0
        self.obs_distance_min = 5.0
        self.k_oa = 2.0
        self.delta_x = 100.0
        self.delta_y = 100.0

        self.kp_position = 1.2
        self.kd_position = 0
        self.k_yaw = 0.5
        self.ibvs_x = 0.003
        self.ibvs_y = 0.003
        self.cnt_vs = 0

        # mavros topics
        if is_flight == 1:
            #self.local_pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.local_pose_callback)
            self.local_pose_sub = rospy.Subscriber("mavros/global_position/raw/fix", NavSatFix, self.local_pose_callback)
            self.local_another_pose_sub = rospy.Subscriber("/fromothermaster/global", NavSatFix, self.local_another_pose_callback)
        else:
            self.local_pose_sub = rospy.Subscriber("mavros/global_position/raw/fix", NavSatFix, self.local_pose_callback)
            self.local_another_pose_sub = rospy.Subscriber("/fromothermaster/global", NavSatFix, self.local_another_pose_callback)
        #self.local_vel_sub = rospy.Subscriber("mavros/local_position/velocity_local", TwistStamped, self.local_vel_callback)
        # self.mavros_sub = rospy.Subscriber("mavros/state", State, self.mavros_state_callback)
        #self.rcin_sub = rospy.Subscriber("mavros/rc/in", RCIn, self.rcin_callback)
        self.local_vel_pub =  rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        #self.vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)                     # 发布速度指令（重要）
        #self.pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)                        # 发布位置指令（初始化飞到厂房前使用）
        #self.VS_end_pub = rospy.Publisher('VS_end', Bool, queue_size=1)
        #self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)                                              # 解锁服务
        #self.flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)                                             # 飞行模式服务
        #self.takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)                                          # 起飞服务
        print("Avoidence Initialized with {}".format(self.drone_name))

        # 无人机位置姿态回调函数
    '''
    def local_pose_callback(self, msg):
        if not self.is_initialize_pos:
            self.mav_yaw_0 = self.mav_yaw
        self.is_initialize_pos = True
        self.mav_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        q0, q1, q2, q3 = msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z
        self.mav_yaw = np.arctan2(2*(q0*q3 + q1*q2), 1-2*(q2*q2 + q3*q3))
        self.mav_R = np.array([
            [q0**2+q1**2-q2**2-q3**2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2)],
            [2*(q1*q2+q0*q3), q0**2-q1**2+q2**2-q3**2, 2*(q2*q3-q0*q1)],
            [2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0**2-q1**2-q2**2+q3**2]
        ])
    '''
    #无人机GPS位置回调函数
    def local_pose_callback(self, msg):
        if not self.is_initialize_pos:
            self.mav_yaw_0 = self.mav_yaw
        self.is_initialize_pos = True
        self.mav_pos = np.array([msg.latitude, msg.longitude, msg.altitude])
        
    
    # another 无人机位置姿态回调函数
    def local_another_pose_callback(self, msg):
        #if not self.is_initialize_pos:
        #    self.mav_yaw_0 = self.mav_yaw
        #self.is_initialize_pos = True
        '''
        EARTH_RADIUS1 = 6371000
        deg2rad = np.pi / 180.0
        self.another_mav_pos = np.array([msg.latitude, msg.longitude, msg.altitude])
        lat1 = self.mav_pos[0] * deg2rad
        lat2 = self.another_mav_pos[0] * deg2rad
        lng1 = self.mav_pos[1] * deg2rad
        lng2 = self.another_mav_pos[1] * deg2rad
        a = lat1 - lat2
        b = lng1 - lng2
        s = 2 * np.sin(np.sqrt(np.power(np.sin(a / 2), 2) + np.cos(lat1) * np.cos(lat2) * np.power(np.sin(b / 2), 2)))
        s = s * EARTH_RADIUS1
        self.obs_distance = s
        self.delta_x = a * EARTH_RADIUS1
        self.delta_y = b * EARTH_RADIUS1
        '''
        deg2rad = np.pi / 180.0
        self.another_mav_pos = np.array([msg.latitude, msg.longitude, msg.altitude])
        x = (self.mav_pos[1] - self.another_mav_pos[1])*111318.0*np.cos((self.mav_pos[0] + self.another_mav_pos[0])/2*deg2rad)
        y = (self.mav_pos[0] - self.another_mav_pos[0])*110946.0
        self.obs_distance = np.sqrt(x**2 + y**2)
        self.delta_x = x
        self.delta_y = y
        # print('distance:{:.2f}, delta_x:{:.2f}, delta_y:{:.2f}'.format(self.obs_distance,x,y))
        #print('delta_y',y)
        #gain needs to be same scale jiaxing sugpest to use accurate equatior
        #q0, q1, q2, q3 = msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z
        #self.another_mav_yaw = np.arctan2(2*(q0*q3 + q1*q2), 1-2*(q2*q2 + q3*q3))
        #self.another_mav_R = np.array([
        #    [q0**2+q1**2-q2**2-q3**2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2)],
        #    [2*(q1*q2+q0*q3), q0**2-q1**2+q2**2-q3**2, 2*(q2*q3-q0*q1)],
        #    [2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0**2-q1**2-q2**2+q3**2]
        #])
        #self.obs_distance_2 = (self.mav_pos[0] - self.another_mav_pos[0])**2 + (self.mav_pos[1] - self.another_mav_pos[1])**2 + (self.mav_pos[2] - self.another_mav_pos[2])**2
        #self.local_obs_avoidance()

    def saturation(self, a, maxv):
        n = np.linalg.norm(a)
        if n > maxv:
            return a / n * maxv
        else:
            return a

    def local_obs_avoidance(self):
        velocity = 10
        if self.obs_distance < self.obs_distance_min:
            # print('Obstacle Avoidance!')
            # print('Another drone pos:',self.another_mav_pos)
            # print('My drone pos:',self.mav_pos)
            p_d = np.array([self.delta_x , self.delta_y])
            cmd_v = self.k_oa * (self.obs_distance_min - np.linalg.norm(p_d)) * p_d/np.linalg.norm(p_d)
            cmd_v[1] = 1.5 * cmd_v[1]
            cmd_v = self.saturation(cmd_v, velocity)
            #cmd_yaw = self.saturation(self.k_yaw * self.minAngleDiff(yaw_d, self.mav_yaw), 0.4)
            # print('cmd_v: ',cmd_v[0],cmd_v[1])

            self.moveByVelocityYawrateENU(cmd_v[0], cmd_v[1], 0., 0.)
    
    def moveByVelocityYawrateENU(self, E=0, N=0, U=0, yaw_rate=0):
        command_vel = construct_veltarget_ENU(E, N, U,yaw_rate)
        self.local_vel_pub.publish(command_vel)

    

    
def construct_veltarget_ENU(E=0, N=0, U=0, yaw_rate=0):
    target_raw_pose = PositionTarget()
    target_raw_pose.header.stamp = rospy.Time.now()
    
    '''
    uint8 FRAME_LOCAL_NED = 1
    uint8 FRAME_LOCAL_OFFSET_NED = 7
    uint8 FRAME_BODY_NED = 8
    uint8 FRAME_BODY_OFFSET_NED = 9
    '''
    # 直接就ENU了
    target_raw_pose.coordinate_frame = 1

    target_raw_pose.velocity.x = E
    target_raw_pose.velocity.y = N
    target_raw_pose.velocity.z = U

    target_raw_pose.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ \
                                + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                                + PositionTarget.FORCE + PositionTarget.IGNORE_YAW 

    target_raw_pose.yaw_rate = yaw_rate
    return target_raw_pose