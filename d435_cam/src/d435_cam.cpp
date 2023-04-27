#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/videoio.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

using namespace std;
using namespace cv;
// Board
// const int lowh = 174;
// const int lows = 130;
// const int lowv = 100;
// const int highh = 181;
// const int highs = 220;
// const int highv = 255;
// Ballon
const int lowh = 171;
const int lows = 240;
const int lowv = 100;
const int highh = 178;
const int highs = 256;
const int highv = 200;
const int lowh2 = 10;
const int lows2 = 120;
const int lowv2 = 80;
const int highh2 = 0;
const int highs2 = 256;
const int highv2 = 256;
ros::Publisher centerPointPub;
bool is_image_initialize = false;
Mat img;


void imageCallback(const sensor_msgs::Image::ConstPtr &imgae_msg)
{
    is_image_initialize = true;
    // ros::Time begin_image = ros::Time::now();
	//red
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(imgae_msg, sensor_msgs::image_encodings::BGR8);
	img = cv_ptr -> image;
	flip(img, img, -1);
}

int main( int argc, char** argv )
{
    ros::init(argc,argv,"d435_camera");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    ros::Rate r(30);

	image_transport::Subscriber color_sub = it.subscribe("/d435i/color/image_raw", 1, imageCallback);
    centerPointPub = nh.advertise<std_msgs::Float32MultiArray>("/tracker/pos_image",1);

    //逐帧显示
    while(ros::ok())
    {
        if (!is_image_initialize) {
            ros::spinOnce();
            r.sleep();
            continue;
        }
        Mat imgHSV;
        vector<Mat> hsvSplit;
        
        cvtColor(img,imgHSV,COLOR_BGR2HSV);
        // split(imgHSV,hsvSplit);
        // equalizeHist(hsvSplit[2],hsvSplit[2]);
        // merge(hsvSplit,imgHSV);
        
        Mat imgThresholded, imgThresholded1, imgThresholded2;
        
        inRange(imgHSV,Scalar(lowh,lows,lowv),Scalar(highh,highs,highv),imgThresholded1);
        inRange(imgHSV,Scalar(lowh2,lows2,lowv2),Scalar(highh2,highs2,highv2),imgThresholded2);
        imgThresholded = imgThresholded1 + imgThresholded2;
        Mat element = getStructuringElement(MORPH_RECT,Size(2,2));
        morphologyEx(imgThresholded,imgThresholded,MORPH_RECT,element);
        
        Point3d xy;
        double m00, m10, m01;
        Moments moment;
        moment = moments(imgThresholded, true);
        m00 = moment.m00; //cvGetSpatialMoment( &moment, 0, 0 );
        if( m00 >= 5) 
        {
            m10 =moment.m10;// cvGetSpatialMoment( &moment, 1, 0 );
            m01 = moment.m01;//cvGetSpatialMoment( &moment, 0, 1 );
            xy.x = (int) (m10/m00);
            xy.y = (int) (m01/m00);
            // std::cout<<xy.x<<","<<xy.y<<endl;
            float zhiling = 0.0;
            std_msgs::Float32MultiArray msg;
            msg.data.push_back(xy.x);
            msg.data.push_back(xy.y);
            msg.data.push_back(zhiling);
            msg.data.push_back(zhiling);
            msg.data.push_back(zhiling);
            centerPointPub.publish(msg);

            // circle(img,cvPoint(xy.x,xy.y),3,CV_RGB(0,0,255),5);
        }
        else{
            // std::cout<<" ------>>m00 = 0"<<std::endl;
            std_msgs::Float32MultiArray msg;
            msg.data.push_back(-1.0);
            msg.data.push_back(-1.0);
            msg.data.push_back(-1.0);
            msg.data.push_back(-1.0);
            msg.data.push_back(-1.0);
            centerPointPub.publish(msg);
        }

        // /* Shown image on screen */
        // Mat mask_rgb;
        // vector<Mat> imgThresholdedVec = {imgThresholded, imgThresholded, imgThresholded};
        // merge(imgThresholdedVec, mask_rgb);
        // imshow( "imgThresholded", mask_rgb );
        // imshow("CSI Camera",img);
        // cv::waitKey(1);

        ros::spinOnce();
        r.sleep();
    }
    
}
