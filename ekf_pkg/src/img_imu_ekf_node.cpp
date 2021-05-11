#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>

#include <Eigen/Geometry>
#include <deque>

#include "img_imu_ekf.h"
#include <algorithm>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>

#include "sensor_msgs/image_encodings.h"
#include <opencv/cv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core/core.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.h>

cv::Mat imgCallback;

using namespace std;
using namespace Eigen;

// roslaunch ekf_pkg img_ekf.launch | tee -a `roscd ekf_pkg/log/ && pwd`/`date +%Y%m%d_%H%M%S_fly.log`
img_imu_ekf img_imu;
sensor_msgs::Image img_predict;

double pos_ref_time = 0;
double vel_ref_time = 0;
double imu_ref_time = 0;
double imu_pre_time = 1.61916e+09;//这个为系统时钟
double last_time = 0; //这个为系统时钟
double img_ref_time = 0;
bool time_flag = false;

Vector3d acc;
Vector3d gyro;
Vector2d mav_img;
Vector2d img0(360,202.5);//图像分别率
int img_f = 360;
Vector3d mav_vel;
Vector3d mav_pos;
Vector3d target_pos(-5,15,1);
Vector4d mav_q;
Vector4d mav_qq;

int loop_cnt = 0;
bool is_img_come = false;
bool get_img = false;
bool get_orientation = false;
bool get_vel = false;
bool get_sensor = false;

MatrixXd Phi[50];
VectorXd ZZ[50];
MatrixXd GG[50];
Vector3d ACC[50];
MatrixXd PP;//有Img后，第一次使用的P
MatrixXd KK;
// MatrixXd GG;

ros::Publisher pub_img_pos;


//predict(MatrixXd Phi, MatrixXd X, MatrixXd P, MatrixXd G, MatrixXd Q)
//void update(MatrixXd P, MatrixXd H, MatrixXd R, MatrixXd X, MatrixXd K, MatrixXd Phi, VectorXd Z);


//Img来了之后进行循环,传入的参数为当前循环周期
void loop_img(int loop)
{
    // MatrixXd GG; //定义单位矩阵
    
    //img来了之后，迭代n次
    //第一次迭代只有更新步骤，没有预测步骤
    //PP为第一次img没来，imu产生的协方差预测
    
    if(loop == 1)
    {
        cout << "loop_img step1 is success!!!!" << endl;
        cout << "Matrix H:" << img_imu.kf.H << endl;
        cout << "Matrix PP:" << PP << endl;
        cout << "Matrix QQ:" << img_imu.R << endl;
        KK = PP * img_imu.kf.H.transpose() * (img_imu.kf.H * PP * img_imu.kf.H.transpose() + img_imu.R).inverse();
        cout << "loop KK success!!!!" << endl;
        img_imu.kf.update(PP, img_imu.kf.H, img_imu.Q, img_imu.kf.x, KK, ZZ[loop_cnt]);//状态观测为图像来临时的观测
        cout << "loop step1 success!!!!" << loop << endl;
        // cout << "Phi:" << img_imu.Phi << endl;
        // cout << "Covariance:" << img_imu.kf.P << endl;
        // cout << "R:" << img_imu.R << endl;
        // cout << "State:" << img_imu.kf.x << endl;
        // cout << "k_10:" << KK(10,10) << endl;
        // cout << "k_11:" << KK(11, 11) << endl;
    }
    else
    {
        cout << "loop_img step2 is success!!!!" << endl;
        // img_imu.kf.predict(Phi[loop - 1], img_imu.kf.x, img_imu.kf.P, GG[loop - 1], img_imu.Q);
        img_imu.kf.predict(Phi[loop], img_imu.kf.x, img_imu.kf.P, GG[loop], img_imu.Q);
        if (loop == loop_cnt)
        {
            img_imu.Phi = Phi[loop];
        }
        cout << "loop step2 success!!!!" << loop << endl;
        // cout << "Phi:" << img_imu.Phi << endl;
        // cout << "Covariance:" << img_imu.kf.P << endl;
        // cout << "R:" << img_imu.R << endl;
        // cout << "State:" << img_imu.kf.x << endl;
        // cout << "k_11:" << KK(11, 11) << endl;
    }
    cout << "loop:" << loop << endl;
}


void mav_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // cout << "Mav_pos is ok!" << endl;
    pos_ref_time = msg->header.stamp.toSec();
    mav_pos(0) = msg->pose.position.x;
    mav_pos(1) = msg->pose.position.y;
    mav_pos(2) = msg->pose.position.z;
    get_orientation = true;
    // current_state = *msg;
}

void mav_vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    get_vel = true;
    // cout << "Mav_vel is ok!" << endl;
    vel_ref_time = msg->header.stamp.toSec();
    mav_vel(0) = msg->twist.linear.x;
    mav_vel(1) = msg->twist.linear.y;
    mav_vel(2) = msg->twist.linear.z;
}

void mav_imu_cb(const sensor_msgs::Imu::ConstPtr &msg)
{
    
    if(get_img && get_orientation && get_vel)
    {
        get_sensor = true;
    }
    cout << "Imu is subcribing!!!!!" << endl;
    cout << "get_sensor：" << get_sensor << endl;
    // cout << "is_img_come state:" << is_img_come << endl;
    Matrix3d I3 = Matrix3d::Identity(); //定义单位矩阵

    imu_ref_time = msg->header.stamp.toSec();

    if (time_flag == false)
    {
        last_time = imu_ref_time;
        imu_pre_time = imu_ref_time;
        time_flag = true;
    }
    cout << "time:" << imu_ref_time - last_time << endl;

    acc(0) = msg->linear_acceleration.x;
    acc(1) = msg->linear_acceleration.y;
    acc(2) = msg->linear_acceleration.z;
    gyro(0) = msg->angular_velocity.x;
    gyro(1) = msg->angular_velocity.y;
    gyro(2) = msg->angular_velocity.z;
    mav_q(0) = msg->orientation.w;
    mav_q(1) = msg->orientation.x;
    mav_q(2) = msg->orientation.y;
    mav_q(3) = msg->orientation.z;
    cout << "mav_q" << mav_q << endl;

    if (img_imu.is_init_done == false && get_sensor == true)
    {
        //初始化的时候应该更新状态X:img_imu.kf.x
        img_imu.sensor_init(mav_q, -mav_pos + target_pos, mav_vel, mav_img);
        //初始化的时候应该更新下Phi:img_imu.Phi
        img_imu.update_Phi(gyro, mav_vel, acc, imu_ref_time - imu_pre_time);
        is_img_come = false;//防止未初始化时，Img先于Imu来临。
    }
    cout << "img_imu is init:" << img_imu.is_init_done << endl;
    cout << "Step1!!!!:" << endl;
    if (img_imu.is_init_done == true && get_sensor == true)
    {
        //执行矫正过程
        if (is_img_come)
        {
            loop_cnt++;
            img_imu.update_Phi(gyro, mav_vel, acc, imu_ref_time - imu_pre_time);
            Phi[loop_cnt] = img_imu.Phi;
            GG[loop_cnt] = img_imu.G;
            ACC[loop_cnt] = acc;
            ZZ[loop_cnt] = VectorXd::Ones(2);
            ZZ[loop_cnt].segment(0, 2) = mav_img;
            // ZZ[loop_cnt].segment(0, 4) = mav_q;
            // ZZ[loop_cnt].segment(4, 3) = -mav_pos + target_pos;
            // ZZ[loop_cnt].segment(7, 3) = mav_vel;
            // ZZ[loop_cnt].segment(10, 2) = mav_img;
            // ZZ[loop_cnt].segment(12, 3) = Vector3d::Ones();
            // ZZ[loop_cnt].segment(15, 3) = Vector3d::Ones();
            // while (loop_cnt--)
            for (int circle = 1; circle <= loop_cnt; circle++)
            {
                loop_img(circle);
            }
            is_img_come = false;
            loop_cnt = 0;
            cout << "Img has been dealing!" << endl;
        }
        else
        {
            if(loop_cnt == 0)
            {
                loop_cnt++;
                // img_imu.kf.predict(img_imu.Phi, img_imu.kf.x, img_imu.kf.P, img_imu.G, img_imu.Q);  Test
                //Phi是上一时刻获得的，因此要先进行预测步骤，在更新Phi，同时还需要保存本次更新的Phi
                cout << "Predict is entering!!!!" << endl;
                img_imu.kf.predict(img_imu.Phi, img_imu.kf.x, img_imu.kf.P, img_imu.G, img_imu.Q);
                //保存这一次的预测协方差阵P,当图像来临时，第一次预测更新用这个协方差阵进行计算
                PP = img_imu.kf.P;
                img_imu.update_Phi(gyro, mav_vel, acc, imu_ref_time - imu_pre_time);
                Phi[loop_cnt] = img_imu.Phi;
                GG[loop_cnt] = img_imu.G;
                cout << "Predict is success!!!!" << endl;
            }
            else
            {
                loop_cnt++;
                img_imu.update_Phi(gyro, mav_vel, acc, imu_ref_time - imu_pre_time);
                Phi[loop_cnt] = img_imu.Phi;
                GG[loop_cnt] = img_imu.G;
                cout << "update_Phi is success!!!!" << endl;
            }
            // loop_cnt++;
            
            // if (loop_cnt == 8)
            // {
            //     is_img_come = true;
            // }
        }
        cout << "loop_cnt:" << loop_cnt << endl;
        imu_pre_time = imu_ref_time;
        img_predict.width = img_imu.kf.x(10);
        img_predict.height = img_imu.kf.x(11);
        cout << "ekf_x:" << img_imu.kf.x(10) * img_f + img0(0) << endl;
        cout << "ekf_y:" << img_imu.kf.x(11) * img_f + img0(1) << endl;
        // pub_img_pos.publish(img_predict);
    }
    
}

void mav_img_cb(const sensor_msgs::Image::ConstPtr &msg)
{
    cout << "IMG is coming!!!!!" << endl;
    is_img_come = true;
    img_ref_time = msg->header.stamp.toSec();
    mav_img(0) = (msg->width - img0(0)) / img_f;
    mav_img(1) = (msg->height - img0(1)) / img_f;
    get_img = true;
    cout << "img_x:" << mav_img(0) * img_f + img0(0) << endl;
    cout << "img_y:" << mav_img(1) * img_f + img0(1) << endl;
}

void img_show_cb(const sensor_msgs::CompressedImage::ConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr_compressed = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    imgCallback = cv_ptr_compressed->image;
    if (get_sensor)
    {
        cv::circle(imgCallback, cv::Point(img_imu.kf.x(10) * img_f + img0(0), img_imu.kf.x(11) * img_f + img0(1)), 5, cv::Scalar(255, 0, 0), 2);
        cv::circle(imgCallback, cv::Point(mav_img(0) * img_f + img0(0), mav_img(1) * img_f + img0(1)), 5, cv::Scalar(255, 255, 0), 1);
    }
    // cv::circle(imgCallback, cv::Point(360, 210), 5, cv::Scalar(255, 0, 0), 2);

    cv::imshow("imgCallback", imgCallback);
    cv::waitKey(1);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ekf_node");
    ros::NodeHandle ekf_img;

    ros::Subscriber pos_sub = ekf_img.subscribe<geometry_msgs::PoseStamped>
                ("mavros/local_position/pose", 10, mav_pose_cb);   //50hz
    ros::Subscriber vel_sub = ekf_img.subscribe<geometry_msgs::TwistStamped>
                ("mavros/local_position/velocity_local", 10, mav_vel_cb);//50hz
    // ros::Subscriber imu_sub = ekf_img.subscribe<sensor_msgs::Imu>
    //             ("/mavros/imu/data", 10, mav_imu_cb);             //50hz
    ros::Subscriber imu_sub = ekf_img.subscribe<sensor_msgs::Imu>
                ("/mavros/imu/data", 10, mav_imu_cb); //50hz
    ros::Subscriber img_sub = ekf_img.subscribe<sensor_msgs::Image>
                ("tracker/pos_image", 10, mav_img_cb);          //23hz
    ros::Subscriber img_show_sub = ekf_img.subscribe<sensor_msgs::CompressedImage>
                ("/camera/left/compressed", 10, img_show_cb); //21hz

    ros::Publisher pub_img_pos = ekf_img.advertise<sensor_msgs::Image>
                ("tracker/pos_image_ekf", 10);

    ros::spin();

    return 0;
}
