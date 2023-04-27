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
const int lowh = 169;
const int lows = 80;
const int lowv = 80;
const int highh = 181;
const int highs = 256;
const int highv = 256;
const int lowh2 = 0;
const int lows2 = 120;
const int lowv2 = 80;
const int highh2 = 5;
const int highs2 = 256;
const int highv2 = 256;
ros::Publisher centerPointPub;

string gstreamer_pipeline (int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method)
{
    return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + to_string(capture_width) + ", height=(int)" +
           to_string(capture_height) + ", format=(string)NV12, framerate=(fraction)" + to_string(framerate) +
           "/1 ! nvvidconv flip-method=" + to_string(flip_method) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}

int main( int argc, char** argv )
{
    ros::init(argc,argv,"csi_camera");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher img_pub = it.advertise("image_raw", 10);
    ros::Time::init();

    centerPointPub = nh.advertise<std_msgs::Float32MultiArray>("/tracker/pos_image",1);

    int capture_width = 640 ;
    int capture_height = 360 ;
    int display_width = 480 ;
    int display_height = 640 ;
    int framerate = 60 ;
    int flip_method = 0 ;

    //创建管道
    string pipeline = gstreamer_pipeline(capture_width,
    capture_height,
    display_width,
    display_height,
    framerate,
    flip_method);
    std::cout << "使用gstreamer管道: \n\t" << pipeline << "\n";

    //管道与视频流绑定
    VideoCapture cap(pipeline, CAP_GSTREAMER);
    if(!cap.isOpened())
    {
        std::cout<<"打开摄像头失败."<<std::endl;
        return (-1);
    }

    //创建显示窗口
    // namedWindow("CSI Camera", WINDOW_AUTOSIZE);
    Mat img_origin, img;

    //逐帧显示
    while(ros::ok())
    {
        if (!cap.read(img_origin))
        {
            std::cout<<"捕获失败"<<std::endl;
            break;
        }
        transpose(img_origin, img);
        flip(img, img, 0);

        Mat imgHSV;
        vector<Mat> hsvSplit;
        
        cvtColor(img,imgHSV,COLOR_BGR2HSV);
        split(imgHSV,hsvSplit);
        equalizeHist(hsvSplit[2],hsvSplit[2]);
        merge(hsvSplit,imgHSV);
        
        Mat imgThresholded, imgThresholded1, imgThresholded2;
        
        inRange(imgHSV,Scalar(lowh,lows,lowv),Scalar(highh,highs,highv),imgThresholded1);
        inRange(imgHSV,Scalar(lowh2,lows2,lowv2),Scalar(highh2,highs2,highv2),imgThresholded2);
        imgThresholded = imgThresholded1 + imgThresholded2;
        Mat element = getStructuringElement(MORPH_RECT,Size(2,2));
        morphologyEx(imgThresholded,imgThresholded,MORPH_RECT,element);
        // std::cout<<"111111111111111111"<<std::endl;
        // imshow("imgThresholded",imgThresholded);
        
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


        cv_bridge::CvImage out_msg;
        out_msg.header.stamp = ros::Time::now();
        out_msg.encoding = sensor_msgs::image_encodings::BGR8;
        out_msg.image = img;
        img_pub.publish(out_msg.toImageMsg());


        // /* Shown image on screen */
        // imshow( "imgThresholded", imgThresholded );
        // imshow("CSI Camera",img);


        // int keycode = cv::waitKey(1) & 0xff ; //ESC键退出
        // if (keycode == 27) 
        //     break ;
        // ros::spinOnce();
    }
    
    cap.release();
    destroyAllWindows() ;
}
