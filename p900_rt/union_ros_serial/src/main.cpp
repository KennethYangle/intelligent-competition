#include<iostream>
#include <ros/ros.h>
#include <serial/serial.h>
#include "uart.h"

int main(int argc, char** argv)
{
    ros::init(argc,argv,"union_ros_serial");
    ros::NodeHandle nh;

    UART uart;
    uart.setPort(uart._port);
    uart.setBaudrate(uart._baudrate);
    uart.setTimeout(uart._to);
    uart.RunMain();

    return 0;
}
