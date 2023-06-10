#include "uart.h"
#include <stdio.h>
using namespace std;

UART::UART():Serial()
{
    ros::NodeHandle nh("~");
    nh.param<std::string>("Port",_port,"/dev/P900");
    cout << "Port:" << _port << endl;
    nh.param<int>("Baudrate",_baudrate,57600);
    cout << "Baudrate:" << _baudrate << endl;
    nh.param<int>("mav_id", _mav_id, 1);
    cout << "mav_id:" << _mav_id << endl;
    _to = Timeout::simpleTimeout(100);

}

void UART::state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void UART::altitude_callback(const mavros_msgs::Altitude &msg)
{
    current_altitude = msg.relative*100;
}

void UART::odometry_callback(const nav_msgs::Odometry &current_info)
{
    // current_altitude = current_info.pose.pose.position.z*100;

    current_angle.x= current_info.pose.pose.orientation.x;
    current_angle.y= current_info.pose.pose.orientation.y;
    current_angle.z= current_info.pose.pose.orientation.z;
    current_angle.w= current_info.pose.pose.orientation.w;
    curr_angle = toEulerAngle(current_angle);
}
void UART::velocity_callback(const geometry_msgs::TwistStamped &msg)
{
    current_vx = msg.twist.linear.x*100;
    current_vy = msg.twist.linear.y*100;
}

void UART::rcin_callback(const mavros_msgs::RCIn & rcvalue)
{
    _channel5_value = rcvalue.channels[4];
    _channel6_value = rcvalue.channels[5];
    _channel7_value = rcvalue.channels[6];
    _channel8_value = rcvalue.channels[7];
}

void UART::gps_callback(const sensor_msgs::NavSatFix &msg)
{
    int current_latitude = msg.latitude*1e7;
    int current_longitude = msg.longitude*1e7;
    // current_altitude = msg.altitude*100;
    //std::cout<<"point1"<<std::endl;
    Int2char current_latitude_union, current_longitude_union, current_altitude_union, current_yaw_union, current_vx_union, current_vy_union;
    current_latitude_union.Int = current_latitude;
    current_longitude_union.Int = current_longitude;
    current_altitude_union.Int = current_altitude;
    current_yaw_union.Int = curr_angle.z*100;
    current_vx_union.Int = current_vx;
    current_vy_union.Int = current_vy;

    //header and payload

    
    uint8_t  temp1[38] = {0xb5,0x62,0xa1,0xb2,0xa1,0x26, 0x11,0x11,0x11,0x11};


    temp1[10] = current_latitude_union.Char[0];
    temp1[11] = current_latitude_union.Char[1];
    temp1[12] = current_latitude_union.Char[2];
    temp1[13] = current_latitude_union.Char[3];

    temp1[14] = current_longitude_union.Char[0];
    temp1[15] = current_longitude_union.Char[1];
    temp1[16] = current_longitude_union.Char[2];
    temp1[17] = current_longitude_union.Char[3];

    temp1[18] = current_altitude_union.Char[0];
    temp1[19] = current_altitude_union.Char[1];
    temp1[20] = current_altitude_union.Char[2];
    temp1[21] = current_altitude_union.Char[3];
    //std::cout<<"point4"<<std::endl;
    //yaw
    temp1[22] = current_yaw_union.Char[0];
    temp1[23] = current_yaw_union.Char[1];
    temp1[24] = current_yaw_union.Char[2];
    temp1[25] = current_yaw_union.Char[3];

    //vx
    temp1[26] = current_vx_union.Char[0];
    temp1[27] = current_vx_union.Char[1];
    temp1[28] = current_vx_union.Char[2];
    temp1[29] = current_vx_union.Char[3];

    //vy
    temp1[30] = current_vy_union.Char[0];
    temp1[31] = current_vy_union.Char[1];
    temp1[32] = current_vy_union.Char[2];
    temp1[33] = current_vy_union.Char[3];


    //temp1[7]-> change the flight formation mode ,set 3 different mode
    // if(_channel7_value<1400)
    // {
    //     temp1[7] = 0x01;
    // }
    // else if(_channel7_value==1500)
    // {
    //     temp1[7] = 0x02;
    // }
    // else if(_channel7_value>1600)
    // {
    //     temp1[7] = 0x03;
    // }

    temp1[7] = 0x01;

    //temp1[8] -> the number of slaver to takeoff
    // if(_channel7_value <1400)
    // {
    //     takeoff_number = 0;
    // }
    // if(_channel7_value <1800)
    // {
    //     channel7_flag = false;
    // }

    // if(_channel7_value >1800 && channel7_flag==false)
    // {
    //     channel7_flag = true;
    //     takeoff_number++;
    // }
    // uint8_t b1 = (uint8_t)takeoff_number;
    //std::cout<<"point3"<<std::endl;
    //temp1[8] -> if the slaver to take off or land: 0x01(takeoff) 0x02(land) 0x03(do nothing)
    if(current_state.mode == "OFFBOARD"&&current_state.armed)
    {
        temp1[8] = 0x01;
    }
    else if(current_state.mode == "AUTO.LAND")
    {
        temp1[8] = 0x02;
    }
    else
    {
       temp1[8] = 0x03;
    }
    
    //temp1[9] -> vehicle id
    temp1[9] = 0x01;
    //command list: 0x01->主机pub    0x02->从机pub   
    temp1[34] = 0x01; //command prepared

    uint8_t crc = 0;

    for(uint8_t i=0;i<35;i++)
    {
        crc += temp1[i];
    }
    temp1[35] = crc;


    // crc check
    temp1[36] = 0x0d;
    temp1[37] = 0x0a;

    
    write(temp1, 38);
    // std::cout<<"publish once dd.... "<<std::endl;
}




void UART::OpenSerial()
{
    try {
        this->open();
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Unable to open port.");
        return;
    }

    if(this->isOpen())
    {
        ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");
    }
    else
    {
        return;
    }
}

uint8_t UART::CrcCheck(std::vector<uint8_t> buff,int len)
{
    uint8_t crc = 0;

    for(uint8_t i=0;i<len;i++)
    {
        crc += buff[i];
    }

    return crc;
}

geometry_msgs::Vector3 UART::toEulerAngle(geometry_msgs::Quaternion quat)
{
    geometry_msgs::Vector3 ans;

    tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
    R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
    return ans;
}


void UART::RunMain()
{   
    
    uint8_t package[256];

    uint8_t rx_head = 0;
    uint8_t rx_wr_index = 0;
    uint8_t b_rx_over = 0;
    serial_process_t _serial_process;
    OpenSerial();
    ros::Rate loop_rate(60);

    ros::NodeHandle nh2;
    //advertise是接收串口数据，发送到ros
    //subscribe是接收ros数据，发送到串口
    ros::Subscriber _global_traj_sub2;
    ros::Publisher _global_pub2 = nh2.advertise<sensor_msgs::NavSatFix>("/fromothermaster/global",10);
    // ros::Publisher _global_pub3 = nh2.advertise<sensor_msgs::NavSatFix>("/fromotherfollower/global",10);
    if (_mav_id == 1) {
        _global_traj_sub2 = nh2.subscribe("/mavros/global_position/raw/fix",10,&UART::gps_callback,this);
    }
    ros::Subscriber _altitude_sub2 = nh2.subscribe("/mavros/altitude",10,&UART::altitude_callback,this);
    ros::Subscriber _local_pos_sub2 = nh2.subscribe("/mavros/local_position/odom",10,&UART::odometry_callback,this);
    ros::Subscriber _local_vel_sub2 = nh2.subscribe("/mavros/local_position/velocity_local",10,&UART::velocity_callback,this);
    ros::Subscriber _rcin_sub2 = nh2.subscribe("/mavros/rc/in",10,&UART::rcin_callback,this);
    ros::Subscriber _state_sub = nh2.subscribe("/mavros/state", 10, &UART::state_cb ,this);
    

    while(ros::ok())
    {

    if (_mav_id == 2) {

        //read serial
        size_t n = this->available();
        //std::cout<<"point2"<<std::endl;
        uint8_t buffer[256]; 
        ////std::cout<<"point14"<<std::endl;

        n = this->read(buffer,n);//must! n<=256
        // // if (n >= 255)
        //     // n = 255;
        // size_t n = 255;
        //std::cout << "n: " << n << std::endl;
        ////std::cout<<"point15"<<std::endl;
        uint8_t b[256];
        for(uint8_t i=0;i<n;i++)
        {
            b[i] = buffer[i];
        }
        //std::cout<<"point13"<<std::endl;
        
        for(uint8_t i=0;i<n;i++)
        {
            if(b[i]==0xb5)
            {
                rx_head = 1;
                package[rx_wr_index++] = b[i];
            }
            else if(b[i]==0x62)
            {
                if(rx_head)
                {
                    rx_wr_index = 0;
                    b_rx_over = 0;
                }
                else
                {
                    package[rx_wr_index++] = b[i];
                    rx_head = 0;
                }

            }
            else
            {
                package[rx_wr_index++] = b[i];
                rx_head = 0;
                if(rx_wr_index>32) //>4*8 zanding
                {
                    if((rx_wr_index+2) == package[3] )
                    {

                        // std::cout<<"get the whole package."<<std::endl;
                        b_rx_over = 1;

                        _serial_process.rx_buffer.clear();
                        _serial_process.rx_buffer.push_back(0xb5);
                        _serial_process.rx_buffer.push_back(0x62);

                        for(uint8_t i=2; i<package[3];i++)
                        {
                            _serial_process.rx_buffer.push_back(package[i-2]);
                        }
                        _serial_process.index = package[3];
                        _serial_process.rx_over = 1;
                        memset(package,0,256);
                    }
                }
            }
            if(rx_wr_index>200)
                rx_wr_index=200;
        }
        
        //std::cout<<"point12"<<std::endl;
        
        if(_serial_process.rx_over==1)
        {
            std::vector<uint8_t> buffer;
            for(uint8_t i=0;i<_serial_process.index;i++)
            {

                buffer.push_back(_serial_process.rx_buffer[i]);
            }
            // std::cout<<(int)buffer[0]<<std::endl;
            if(buffer[0]==0xb5 && buffer[1]==0x62 && buffer[_serial_process.index-2]==0x0d && buffer[_serial_process.index-1]==0x0a)
            {
                // std::cout<<"check header successfully!"<<std::endl;
                //crc check
                uint8_t crc = CrcCheck(buffer,_serial_process.index-3);
                if(crc == buffer[_serial_process.index-3])
                {
                    uint8_t temp[_serial_process.index];
                    for(uint8_t i=0;i<_serial_process.index;i++)
                    {
                        temp[i] = buffer[i];
                    }
                    // std::cout<<"check crc successfully!"<<std::endl;

                    memcpy(&_gpsRec,temp+10,25);
                    //publish gps
                    sensor_msgs::NavSatFix gps_pos;
                    gps_pos.position_covariance[0] = temp[9];//_gpsRec.vehicleid;
                    gps_pos.latitude = _gpsRec.latitude/10000000.0;
                    gps_pos.longitude = _gpsRec.longitude/10000000.0;
                    gps_pos.altitude = _gpsRec.altitude/100.0;
                    gps_pos.position_covariance[1] = _gpsRec.yaw/100.0;
                    //position_covariance[0]->master or follower , position_covariance[1]->vehicle id
                    gps_pos.position_covariance[2] = _gpsRec.command;

                    _global_pub2.publish(gps_pos);
                    
                    // if(gps_pos.position_covariance[2] ==1)
                    // {
                    //     _global_pub2.publish(gps_pos);
                    //     std::cout<<"receive message from other master !"<<std::endl;
                    // }
                    // else if(gps_pos.position_covariance[2] ==2)
                    // {
                    //     _global_pub3.publish(gps_pos);
                    //     std::cout<<"receive message from other followers !"<<std::endl;
                    // }

                }
            }

            _serial_process.rx_over = 0;
        }
        //std::cout<<"point12"<<std::endl;
        
    }
        ros::spinOnce();
        loop_rate.sleep();
    }
}
