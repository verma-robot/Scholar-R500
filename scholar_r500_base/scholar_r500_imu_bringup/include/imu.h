#ifndef SCHOLAR_IMU_H
#define SCHOLAR_IMU_H

#include <ros/ros.h>
#include <ros/time.h>
#include <boost/asio.hpp>
#include <sensor_msgs/Imu.h>
#include <iostream>
#include "tf/transform_datatypes.h"//转换函数头文件
namespace scholar_r500_imu
{


    typedef union
    {
        unsigned char u[4];
        float f;

    }uTof;

    class IMU_DRIVER
    {
        public:
	    IMU_DRIVER(std::string port_name, unsigned int port_rate, float g);
	    ~IMU_DRIVER();

            sensor_msgs::Imu imu_data;

            bool init(std:: string port_name, int port_rate, float g);
	    bool readOnce();
                   
            bool read_msg();	
            bool IMU_READY;
                    
                    
	private:
	    ros::Time current_time, last_time;

            boost::asio::io_service iosev;
            boost::asio::serial_port *sp;

            boost::system::error_code ec;
            float PI = 3.1416;

            double acc_g;    
            bool exception;
            uTof temp_data;

    };
    
}

#endif /* SCHOLAR_IMU_H */
