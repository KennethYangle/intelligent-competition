#ifndef UART_H
#define UART_H
#include<iostream>
#include <ros/ros.h>
#include <serial/serial.h>
// #include "unionsys_msgs/TrackRectangle.h"
// #include "unionsys_msgs/Gimbal_PidVel_cmd.h"
#include <vector>
#include <algorithm>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMavFrame.h>

#include <geometry_msgs/TwistStamped.h>

#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h> 
#include <std_msgs/Float32MultiArray.h>
// #include <opencv2/opencv.hpp>
// #include <opencv2/core/core.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <sensor_msgs/LaserScan.h>  qaq
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/Altitude.h>
using namespace serial;
using namespace std;

typedef struct{
    std::vector<uint8_t> rx_buffer;
    uint8_t rx_over;
    uint8_t index;
}serial_process_t;

typedef struct{
    int pos_x;
    int pos_y;
    int pos_z;
    int vel_x;
    int vel_y;
    int vel_z;
    uint8_t command;
} pos_to_pub;
typedef struct{
    int latitude;
    int longitude;
    int altitude;
    int yaw;
    int vx;
    int vy;
    uint8_t command;
} gps_to_pub;

union Int2char
 {
  int Int;
  char Char[4];
 };

class UART : public Serial
{
public:
    UART();
    void OpenSerial();
    void RunMain();
    uint8_t CrcCheck(std::vector<uint8_t> buff, int len);
    void state_cb(const mavros_msgs::State::ConstPtr& msg);
    void altitude_callback(const mavros_msgs::Altitude &msg);
    void odometry_callback(const nav_msgs::Odometry &current_info);
    void velocity_callback(const geometry_msgs::TwistStamped &msg);
    void rcin_callback(const mavros_msgs::RCIn & rcvalue);
    void gps_callback(const sensor_msgs::NavSatFix &msg);
    geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat);
    string DecIntToHexStr(uint8_t num);

public:
    std::string _port;
    int         _baudrate;
    int         _mav_id;
    Timeout     _to;
    int current_altitude = 0;
    int current_vx = 0;
    int current_vy = 0;
    int _channel5_value = 0;
    int _channel6_value = 0;
    int _channel7_value = 0;
    int _channel8_value = 0;
    bool channel7_flag = false;
    int takeoff_number = 0;

    pos_to_pub _positionRec;
    gps_to_pub _gpsRec;

    ros::Publisher  _pose_pub, _traj_pub;
    ros::Subscriber _gimbal_vel_sub, _traj_sub;
    geometry_msgs::Quaternion current_angle;
    geometry_msgs::Vector3 curr_angle;
    mavros_msgs::State current_state;
};

#endif // UART_H
