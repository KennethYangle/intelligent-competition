#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <string>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/objdetect.hpp>
#include <opencv4/opencv2/imgproc/types_c.h>
#include <opencv4/opencv2/videoio.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>


using namespace std;
using namespace cv;
const int lowh = 168;
const int lows = 0;
const int lowv = 0;
const int highh = 181;
const int highs = 255;
const int highv = 255;



string gstreamer_pipeline (int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method)
{
    return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + to_string(capture_width) + ", height=(int)" +
           to_string(capture_height) + ", format=(string)NV12, framerate=(fraction)" + to_string(framerate) +
           "/1 ! nvvidconv flip-method=" + to_string(flip_method) + " ! video/x-raw, width=(int)" + to_string(display_width) + ", height=(int)" +
           to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}




int main( int argc, char** argv )
{
    ros::init(argc,argv,"csi_camera");
    ros::NodeHandle nh;
    ros::Time::init();

image_transport::ImageTransport it(nh);//发布图片需要用到image_transport
        image_transport::Publisher pub_img = it.advertise("/csi_cam/img", 1);

    //centerPointPub = nh.advertise<std_msgs::Float32MultiArray>("/tracker/pos_image",1);

    int capture_width = 1280 ;
    int capture_height = 720 ;
    int display_width = 1280 ;
    int display_height = 720 ;
    int framerate = 60 ;
    int flip_method = 2 ;

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
    Mat img;

    //逐帧显示
        int j=0;
    while(ros::ok())
    {
        j++;
        
        if (!cap.read(img))
        {
            std::cout<<"捕获失败"<<std::endl;
            break;
        }
        //imshow("CSI Camera",img);
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
if (j%1==0){
        pub_img.publish(msg);
//imwrite("/home/nvidia/fisheye_ws/src/csi_cam/src/dat/"+std::to_string(j)+".jpg",img);
}
        
        ros::spinOnce();
        
    }
    

    cap.release();
    destroyAllWindows() ;
}
