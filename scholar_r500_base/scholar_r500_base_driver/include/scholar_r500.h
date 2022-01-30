#ifndef SCHOLAR_R500_ROBOT_H
#define SCHOLAR_R500_ROBOT_H

#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Range.h>
#include "scholar_r500_base_driver/scholar_r500_battery.h"

union motor_speed
{
   unsigned char u[2];
   int16_t value;
};

union sonar_distance
{
   unsigned char u[2];
   uint16_t value;
};

union year
{
   unsigned char u[2];
   uint16_t value;
};

struct motor_data
{
   union motor_speed motor_speed[2];
   unsigned char motor_disconnect[2];
};

struct product_data
{
   union year product_year;
   int8_t product_month;
   int8_t product_day;
   int8_t product_number;

};

namespace scholar_r500
{        
   const float PI = 3.1416;

       
	class scholar_r500_hardware
	{
		public:
		   scholar_r500_hardware(std::string port_name, unsigned int port_rate);
		   ~scholar_r500_hardware();
                   
		   bool init(std::string port_name, unsigned int port_rate); 

         bool spOnce(double linear_speed, double angular_speed);
         bool read_msg();	

         void handle_read( char *buf, boost::system::error_code ec, std::size_t bytes_transferred );
         void listen_data(int buffer_number, int max_seconds);

      public:

		   bool read_flag;
         bool write_falg;
         bool scholar_r500_ready;
         bool read_product_data_flag;
         bool sudden_stop_flag;
         bool disable_sonar_sudden_stop_flag;
         bool enable_sonar_sudden_stop_flag;

         scholar_r500_base_driver::scholar_r500_battery battery;//电池电量信息

         nav_msgs::Odometry odom;//里程计
         sensor_msgs::Range sonar_range[4];
         geometry_msgs::TransformStamped transformStamped;

         struct product_data product_data;//产品序列号

         float scholar_r500_linear_speed_factor;
         float scholar_r500_angular_speed_factor;

         float scholar_r500_wheel_diameter;//轮胎直径
         float scholar_r500_wheel_track;//轴距
         float scholar_r500_max_acc_x;
         float scholar_r500_max_acc_w;


		private:

         void readSpeedCommand();                     
		   void writeSpeedCommand(double linear_speed, double angular_speed);
         void read_product_data();
         void sudden_stop();
         void disable_sonar_sudden_stop();
         void enable_sonar_sudden_stop(uint16_t sonar_sudden_range);
                    
		private:

		   ros::Time current_time, last_time;
         ros::Time write_current_time, write_last_time;


         boost::asio::io_service iosev;
         boost::asio::serial_port *sp;

         boost::system::error_code ec;
                  
         int read_buffer_size;
 
         bool exception;
         //里程计相关

         double x;
		   double y;
		   double th;

		   double vx;
         double vx_old;
		   double vth;
         double vth_old;

         double last_write_vx;
         double last_write_vth;
		 
         union sonar_distance sonar[4];
         struct motor_data motor_data;                    

	};
    
}

#endif /* SCHOLAR_R500_ROBOT_H */
