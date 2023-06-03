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


# 无人机控制类
class Px4Controller:
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
        self.kp_position = 1.2
        self.kd_position = 0
        self.k_yaw = 0.5
        self.ibvs_x = 0.003
        self.ibvs_y = 0.003
        self.cnt_vs = 0

        # mavros topics
        if is_flight == 1:
            self.local_pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.local_pose_callback)
        else:
            self.local_pose_sub = rospy.Subscriber("mavros/local_position/pose_cor", PoseStamped, self.local_pose_callback)
        self.local_vel_sub = rospy.Subscriber("mavros/local_position/velocity_local", TwistStamped, self.local_vel_callback)
        # self.mavros_sub = rospy.Subscriber("mavros/state", State, self.mavros_state_callback)
        self.rcin_sub = rospy.Subscriber("mavros/rc/in", RCIn, self.rcin_callback)
        self.local_vel_pub =  rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        self.vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)                     # 发布速度指令（重要）
        self.pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)                        # 发布位置指令（初始化飞到厂房前使用）
        self.VS_end_pub = rospy.Publisher('VS_end', Bool, queue_size=1)
        self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)                                              # 解锁服务
        self.flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)                                             # 飞行模式服务
        self.takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)                                          # 起飞服务
        print("Px4 Controller Initialized with {}".format(self.drone_name))

    # 任务开始前的一些动作，包括解锁、进offboard、飞到厂房前
    def start(self):
        ## 实飞
        if is_flight == 1:
            for _ in range(10):
                self.vel_pub.publish(self.command)
                # self.arm_state = self.arm()
                # self.offboard_state = self.offboard()
                self.rate.sleep()
            # 不同场景做不同的准备动作
            if self.scene == "32s":             # 主要
                if self.level_id == 1:
                    # print("before position:", self.mav_pos)
                    self.takeoff(h=1.5)
                    # print("after position:", self.mav_pos)
                elif self.level_id == 2:
                    self.takeoff(h=4.5)
                else:
                    print("unknown level!!!")
        
        ## 仿真
        else:
            self.task_ready = True
            for _ in range(10):
                self.vel_pub.publish(self.command)
                self.arm_state = self.arm()
                self.offboard_state = self.offboard()
                self.rate.sleep()
            # 不同场景做不同的准备动作
            if self.scene == "32s":             # 主要
                if self.level_id == 1:
                    self.takeoff_sim(h=1.8)
                elif self.level_id == 2:
                    self.takeoff_sim(h=6.8)
                else:
                    print("unknown level!!!")


        # self.task_ready = True
        while not self.task_ready:
            self.task_ready = (self.ch7 == 2) #and self.offboard_state
            self.mav_pos0 = self.mav_pos
            self.vel_pub.publish(self.command)
            # print("wait task_ready","px.ch7=",px.ch7,"px.offboard_state",px.offboard_state)
            time.sleep(0.5)
        print("\n==============================")
        print("Begin Task!")

        if self.scene == "paper":           # 论文使用
            self.takeoff(h=6)
            des_pos = np.array([0, 35, 3.5])
            dis = np.linalg.norm(des_pos-self.mav_pos)
            command_vel = TwistStamped()
            while dis > 0.5:
                norm_vel = (des_pos-self.mav_pos)/dis*11
                command_vel.twist.linear.x,command_vel.twist.linear.y,command_vel.twist.linear.z = norm_vel
                self.vel_pub.publish(command_vel)
                dis = np.linalg.norm(des_pos-self.mav_pos)
                self.rate.sleep()

        if self.scene == "freeflight":      # test for freeflight.py
            if self.drone_id == 1:
                self.start_point.pose.position.x = 0
                self.start_point.pose.position.y = 0
                self.start_point.pose.position.z = 2
            elif self.drone_id == 2:
                self.start_point.pose.position.x = 4
                self.start_point.pose.position.y = 0
                self.start_point.pose.position.z = 2
            elif self.drone_id == 3:
                self.start_point.pose.position.x = 0
                self.start_point.pose.position.y = 4
                self.start_point.pose.position.z = 2

        if self.scene == "attack_in_oldfactory":      # oldfactory场景起点（以后删）
            self.start_point.pose.position.x = 1
            self.start_point.pose.position.y = -12
            self.start_point.pose.position.z = 2.5
            for _ in range(300):
                self.pos_pub.publish(self.start_point)
                self.rate.sleep()

        if self.scene == "freeflight":      # test for tube
            des_pos = np.array([46.55, 36.75, 1.0])
            self.start_point.pose.position.x = des_pos[0]
            self.start_point.pose.position.y = des_pos[1]
            self.start_point.pose.position.z = des_pos[2]
            dis = np.linalg.norm(des_pos-self.mav_pos)
            while dis > 0.5:
                self.pos_pub.publish(self.start_point)
                dis = np.linalg.norm(des_pos-self.mav_pos)
                self.rate.sleep()

    # 无人机位置姿态回调函数
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

    def local_vel_callback(self, msg):
        self.is_initialize_vel = True
        self.mav_vel = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])

    # def mavros_state_callback(self, msg):
    #     self.mavros_state = msg
    #     self.arm_state = msg.armed
    #     self.mode_str = msg.mode
    #     self.offboard_state = True if msg.mode == "OFFBOARD" else False

    def rcin_callback(self, msg):
        self.is_initialize_rc = True
        last_ch5, last_ch6, last_ch7, last_ch8, last_ch9, last_ch11, last_ch14 = self.ch5, self.ch6, self.ch7, self.ch8, self.ch9, self.ch11, self.ch14
        chs = msg.channels
        # 0-1300->0/1301-1700->1/1701-2000->2
        self.ch5 = 0 if chs[4] < 1300 else 1 if chs[4] < 1700 else 2
        self.ch6 = 0 if chs[5] < 1300 else 1 if chs[5] < 1700 else 2
        self.ch7 = 0 if chs[6] < 1300 else 1 if chs[6] < 1700 else 2
        self.ch8 = 0 if chs[7] < 1300 else 1 if chs[7] < 1700 else 2
        self.ch9 = 0 if chs[8] < 1300 else 1 if chs[8] < 1700 else 2
        self.ch11 = 0 if chs[10] < 1500 else 1
        self.ch14 = 0 if chs[10] < 1500 else 1
        if self.ch5!=last_ch5 or self.ch6!=last_ch6 or self.ch7!=last_ch7 or self.ch8!=last_ch8 or self.ch9!=last_ch9 or self.ch11!=last_ch11 or self.ch14!=last_ch14:
            print("ch5: {}, ch6: {}, ch7: {}, ch8: {}, ch9: {}, ch11: {}, ch14: {}".format(self.ch5, self.ch6, self.ch7, self.ch8, self.ch9, self.ch11, self.ch14))

    # 悬停
    def idle(self):
        print("I'm in idle state!")
        idle_cmd = TwistStamped()
        while not rospy.is_shutdown():
            self.vel_pub.publish(idle_cmd)
            self.rate.sleep()

    # 解锁
    def arm(self):
        if self.armService(True):
            return True
        else:
            print("Vehicle arming failed!")
            return False

    # 上锁
    def disarm(self):
        if self.armService(False):
            return True
        else:
            print("Vehicle disarming failed!")
            return False

    # 进offboard模式
    def offboard(self):
        if self.flightModeService(custom_mode='OFFBOARD'):
            return True
        else:
            print("Vechile Offboard failed")
            return False

    # 起飞
    def takeoff(self, vz=0.8,h=1.8):
        self.mav_yaw_0 = self.mav_yaw

        self.moveByPosENU(U=h)
        takeoff_done = False
        while not takeoff_done:
            self.moveByPosENU(U=h)
            # command = TwistStamped()
            # command.twist.linear.z = vz
            # self.vel_pub.publish(command)
            rospy.sleep(0.05)
            # print("takeoff height:",{self.mav_pos[2]})
            takeoff_done = (abs(self.mav_pos[2]- h) < 0.05)

    # 仿真起飞
    def takeoff_sim(self, vz=0.8,h=1.8):
        self.moveByPosENU(E=0, N=0, U=h)
        takeoff_done = False
        while not takeoff_done:
            self.moveByPosENU(E=0, N=0, U=h)
            # command = TwistStamped()
            # command.twist.linear.z = vz
            # self.vel_pub.publish(command)
            rospy.sleep(0.05)
            # print("takeoff height:",{self.mav_pos[2]})
            takeoff_done = (abs(self.mav_pos[2]- h) < 0.05)

    def moveByPosENU(self, E=None, N=None, U=None, mav_yaw=None):
        mav_yaw = mav_yaw if mav_yaw is not None else self.mav_yaw
        E = E if E is not None else self.mav_pos[0]
        N = N if N is not None else self.mav_pos[1]
        U = U if U is not None else self.mav_pos[2]
        command_vel = construct_postarget_ENU(E, N, U, mav_yaw)
        # print("construct_postarget_ENU:", command_vel)
        self.local_vel_pub.publish(command_vel)
    
    def moveByVelocityYawrateENU(self, E=0, N=0, U=0, yaw_rate=0):
        command_vel = construct_veltarget_ENU(E, N, U,yaw_rate)
        self.local_vel_pub.publish(command_vel)

    def moveToPositionOnceAsync(self, x, y, z, yaw_d, velocity=1):
        """
        - function: 向world ENU坐标系下期望位置和偏航角飞行
        - params:
            - x (float): desired position in world (ENU) X axis
            - y (float): desired position in world (ENU) Y axis
            - z (float): desired position in world (ENU) Z axis
            - yaw_d (float): desired yaw
            - velocity (float): velocity saturation
        - return:
            - 异步执行
        - usage:
            while d < th: update p_d, p  moveToPositionOnceAsync(x, y, z, yaw, v)  time.sleep(t)
        """
        p_d = np.array([x, y, z])
        cmd_v = self.saturation(self.kp_position * (p_d - self.mav_pos) + self.kd_position*(-self.mav_vel), velocity)
        cmd_yaw = self.saturation(self.k_yaw * self.minAngleDiff(yaw_d, self.mav_yaw), 0.4)
        self.moveByVelocityYawrateENU(cmd_v[0], cmd_v[1], cmd_v[2], cmd_yaw)

    def moveByImageError(self, ex, ey, yaw_d, velocity=0.5):
        if ex == 10086:
            v_e = [0, 0, 0]
            cmd_yaw = 0
        else:
            v_b = np.array([0, self.ibvs_x*ex, self.ibvs_y*ey])
            v_e = self.saturation(self.mav_R.dot(v_b), velocity)
            cmd_yaw = self.saturation(self.k_yaw * self.minAngleDiff(yaw_d, self.mav_yaw), 0.4)
        self.moveByVelocityYawrateENU(v_e[0], v_e[1], v_e[2], cmd_yaw)
        # print("ex:", ex, "ey:", ey, "v_e:", v_e)
        if ex < 25 and ey < 30 and np.linalg.norm(self.mav_vel) < 0.2 and abs(self.minAngleDiff(yaw_d, self.mav_yaw))<0.15:
            self.cnt_vs += 1
        else:
            self.cnt_vs = 0
        if self.cnt_vs >= 50:
            a = Bool()
            a.data = True
            self.VS_end_pub.publish(a)
            self.cnt_vs = 0
            print("VS finish")

    def saturation(self, a, maxv):
        n = np.linalg.norm(a)
        if n > maxv:
            return a / n * maxv
        else:
            return a

    def minAngleDiff(self, a, b):
        diff = a - b
        if diff < 0:
            diff += 2*np.pi
        if diff < np.pi:
            return diff
        else:
            return diff - 2*np.pi


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

def construct_postarget_ENU(E=0, N=0, U=0, yaw=0, yaw_rate = 0):
    target_raw_pose = PositionTarget()
    target_raw_pose.header.stamp = rospy.Time.now()
    
    '''
    uint8 FRAME_LOCAL_NED = 1
    uint8 FRAME_LOCAL_OFFSET_NED = 7
    uint8 FRAME_BODY_NED = 8
    uint8 FRAME_BODY_OFFSET_NED = 9

    MAV_FRAME_BODY_FRD
    '''

    target_raw_pose.coordinate_frame = 1

    target_raw_pose.position.x = E
    target_raw_pose.position.y = N
    target_raw_pose.position.z = U

    target_raw_pose.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                                + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                                + PositionTarget.FORCE

    target_raw_pose.yaw = yaw
    target_raw_pose.yaw_rate = yaw_rate
    return target_raw_pose


class Assemble:

    def __init__(self, param_id, px4, image_center):
        self.px4 = px4
        self.image_center = image_center
        self.start_time = time.time()
        
        self.pipeline_cmd = TwistStamped()
        self.dj_cmd = TwistStamped()
        self.obs_cmd = TwistStamped()
        self.dj_action = Action()
        self.fixedpoint = FixedPoint()
        self.VS = Bool()
        self.ex, self.ey = 10086, 10086
        self.VS_yaw_d = 0

        self.Pipeline_cmd_sub = rospy.Subscriber('Pipeline_cmd', TwistStamped, self.Pipeline_cmd_callback)
        self.DJ_cmd_sub = rospy.Subscriber('DJ_cmd', TwistStamped, self.DJ_cmd_callback)
        self.Obs_cmd_sub = rospy.Subscriber('Obs_cmd', TwistStamped, self.Obs_cmd_callback)
        self.Expect_action_sub = rospy.Subscriber('expect_action'+str(param_id), Action, self.Expect_action_callback)
        self.Fixed_point_sub = rospy.Subscriber('/fixed_point', FixedPoint, self.Fixed_point_callback)
        self.pos_image_sub = rospy.Subscriber("tracker/pos_image_win", BoundingBoxes, self.Pos_image_callback)
        self.VS_begin_sub = rospy.Subscriber("VS_begin", Bool, self.VS_begin_callback)
        self.vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)

    def Pipeline_cmd_callback(self, msg):
        self.pipeline_cmd = msg

    def DJ_cmd_callback(self, msg):
        self.dj_cmd = msg

    def Obs_cmd_callback(self, msg):
        self.obs_cmd = msg

    def Expect_action_callback(self, msg):
        self.dj_action = msg

    def Fixed_point_callback(self, msg):
        self.fixedpoint = msg
        print("fixedpoint:", msg)

    def VS_begin_callback(self, msg):
        self.VS = msg

    def Pos_image_callback(self, msg):
        if len(msg.bounding_boxes) >= 1:
            bbox = msg.bounding_boxes[0]
            x = (bbox.xmin + bbox.xmax) / 2
            y = (bbox.ymin + bbox.ymax) / 2
            self.ex = self.image_center[0] - x
            self.ey = self.image_center[1] - y
            self.VS_yaw_d = -bbox.point.z

    def begin_task(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.dj_action.dj == True:
                self.vel_pub.publish(self.dj_cmd)
            elif self.fixedpoint.rush == True:
                x, y, z = self.fixedpoint.pose.position.x, self.fixedpoint.pose.position.y, self.fixedpoint.pose.position.z
                q0, q1, q2, q3 = self.fixedpoint.pose.orientation.w, self.fixedpoint.pose.orientation.x, self.fixedpoint.pose.orientation.y, self.fixedpoint.pose.orientation.z
                yaw_d = np.arctan2(2*(q0*q3 + q1*q2), 1-2*(q2*q2 + q3*q3))
                self.px4.moveToPositionOnceAsync(x, y, z, yaw_d, 1)
            elif self.VS.data == True:
                self.px4.moveByImageError(self.ex, self.ey, self.VS_yaw_d, 1)
            else:
                self.vel_pub.publish(self.pipeline_cmd)
            rate.sleep()

if __name__=="__main__":
    rospy.init_node("assemble", anonymous=True)
    param_id = rospy.get_param("~drone_id")
    level_id = rospy.get_param("~level_id")
    is_flight = rospy.get_param("~is_flight")

    src_path = os.path.join(rospkg.RosPack().get_path("offboard_pkg"), "..")
    setting_file = open(os.path.join(src_path, "settings.json"))
    setting = json.load(setting_file)
    if is_flight == 1:
        image_center = [setting["RealFlight"]["WIDTH"] / 2.0, setting["RealFlight"]["HEIGHT"] / 2.0]
    else:
        image_center = [setting["Simulation"]["WIDTH"] / 2.0, setting["Simulation"]["HEIGHT"] / 2.0]

    # 飞机初始化，解锁、offboard、飞到厂房前
    px4 = Px4Controller(param_id, level_id, is_flight, setting["SCENE"])
    ass = Assemble(param_id, px4, image_center)
    px4.start()
    ass.begin_task()
