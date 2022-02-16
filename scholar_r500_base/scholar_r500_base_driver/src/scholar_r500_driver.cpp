#include <vector>
#include "scholar_r500.h"

namespace scholar_r500
{

  boost::array<double, 36> odom_pose_covariance = {
    {1e-9, 1e-3, 0, 0, 0, 1e-9, 
    1e-3, 1e-3, 0, 0, 0, 1e-9, 
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 
    1e-9, 1e-9, 0, 0, 0, 1e-6}};
  boost::array<double, 36> odom_twist_covariance = {
    {0.02, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0.05}};

scholar_r500_hardware::scholar_r500_hardware(std::string port_name, unsigned int port_rate):sp(NULL)
{  

    sp = new boost::asio::serial_port(iosev);
    if(sp)init(port_name, port_rate);

}

scholar_r500_hardware::~scholar_r500_hardware()
{


     if (sp) 
     {
        sp -> cancel();
        sp -> close();
        delete sp;
     }

     iosev.stop();
     iosev.reset();

     while(read_flag == true)read_flag = false;
     while(write_falg == true)write_falg = false;
     while(read_flag == true)read_flag = false;
     while(write_falg == true)write_falg = false;
     while(read_product_data_flag == true)read_product_data_flag = false;        
     while(sudden_stop_flag == true)sudden_stop_flag = false;
     while(disable_sonar_sudden_stop_flag == true)disable_sonar_sudden_stop_flag = false;
     while(enable_sonar_sudden_stop_flag == true)enable_sonar_sudden_stop_flag = false;
     while(scholar_r500_ready == true)scholar_r500_ready = false; 
              

}

bool scholar_r500_hardware::init(std::string port_name, unsigned int port_rate)
{
     try {

          sp -> open(port_name, ec); //打开串口
          usleep(20000); 

          tcflush(sp->lowest_layer().native_handle(), TCIOFLUSH);//清空串口输入输出缓存

          if(ec)
          {       
               ROS_ERROR( "Can't open serial port") ;
               throw ec;
          }
          //设置串口相关参数
          sp -> set_option(boost::asio::serial_port::baud_rate(115200));
	  sp -> set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
	  sp -> set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
	  sp -> set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
	  sp -> set_option(boost::asio::serial_port::character_size(8));

          ros::Time::init();
	  current_time = ros::Time::now();
	  last_time = ros::Time::now();          
          write_current_time = ros::Time::now();
          write_last_time = ros::Time::now();      

          x = 0.00;
          y = 0.00;
          th = 0.00;

          vx = 0.00;
          vx_old = 0.00;

          vth = 0.00;
          vth_old = 0.00;

          last_write_vx = 0.00;
          last_write_vth = 0.00;

          while(read_flag == true)read_flag = false;
          while(write_falg == true)write_falg = false;
          while(read_product_data_flag == true)read_product_data_flag = false;        
          while(sudden_stop_flag == true)sudden_stop_flag = false;
          while(disable_sonar_sudden_stop_flag == true)disable_sonar_sudden_stop_flag = false;
          while(enable_sonar_sudden_stop_flag == true)enable_sonar_sudden_stop_flag = false;

          while(scholar_r500_ready == true)scholar_r500_ready = false; 
              
          int try_time = 0;
          while(read_product_data_flag == false && try_time < 5)
          {
               read_product_data();//读取生产日期
               listen_data(14,2);//等待读取结果
               try_time++;

          }
          if(read_product_data_flag)while(scholar_r500_ready == false)scholar_r500_ready = true;               
          if(scholar_r500_ready == true)
          {
             //  disable_sonar_sudden_stop();//禁止声波急停
             //  listen_data(8,2);//等待读取结果
              
               try_time = 0;
               while(disable_sonar_sudden_stop_flag == false && try_time < 5)
               {

                    disable_sonar_sudden_stop();//禁止声波急停
                    listen_data(8,2);//等待读取结果
                    try_time++;

               }
          }

          return true;

     }
     catch(...) {

          while(read_flag == true)read_flag = false;
          while(write_falg == true)write_falg = false;
          while(read_product_data_flag == true)read_product_data_flag = false;        
          while(sudden_stop_flag == true)sudden_stop_flag = false;
          while(disable_sonar_sudden_stop_flag == true)disable_sonar_sudden_stop_flag = false;
          while(enable_sonar_sudden_stop_flag == true)enable_sonar_sudden_stop_flag = false;

          while(scholar_r500_ready == true)scholar_r500_ready = false; 
              
          return false;
     }

}


void scholar_r500_hardware::readSpeedCommand()
{

     uint8_t data_read[6] = {0xFF, 0x03, 0x01, 0x06,0x04, 0x6D};   

     boost::asio::write(*sp, boost::asio::buffer(&data_read[0], 6), ec);

}


void scholar_r500_hardware::writeSpeedCommand(double linear_speed, double angular_speed)
{   

     uint8_t data[10] = {0xFF, 0x03, 0x01, 0x0A, 0x03, 0x00, 0x00, 0x00, 0x00, 0x6D};
     uint8_t check[8];
     
     float left_wheel_rate = 0.00;
     float right_wheel_rate = 0.00;
     
     float wheel_rate_average = 0.00;

     union motor_speed motor_speed_temp;
     float acc_x = 0.00;
     float acc_w = 0.00;

     write_current_time = ros::Time::now();
     float delt_write_time = (write_current_time - write_last_time).toSec();
     if(delt_write_time > 0.001)
     {
          acc_x = (linear_speed - vx) / delt_write_time;
          acc_w = (angular_speed - vth) / delt_write_time;
     }

     if(acc_x > scholar_r500_max_acc_x)linear_speed = vx + scholar_r500_max_acc_x * delt_write_time;
     else if(acc_x < -scholar_r500_max_acc_x)linear_speed = vx - scholar_r500_max_acc_x * delt_write_time;
     else linear_speed = linear_speed;

     if(acc_w > scholar_r500_max_acc_w)angular_speed = vth + scholar_r500_max_acc_w * delt_write_time;
     else if(acc_w < -scholar_r500_max_acc_w)angular_speed = vth - scholar_r500_max_acc_w * delt_write_time;
     else angular_speed = angular_speed;

     last_write_vx = linear_speed;
     last_write_vth = angular_speed;

     if(abs(linear_speed) < 0.02 && abs(angular_speed) < 0.05)
     {
          linear_speed = 0.00;
          angular_speed = 0.00;
     }

     right_wheel_rate = (60 * linear_speed + 30 * scholar_r500_wheel_track * angular_speed * scholar_r500_angular_speed_factor) * 10 / (PI * scholar_r500_wheel_diameter * scholar_r500_linear_speed_factor); 
     left_wheel_rate = (60 * linear_speed - 30 * scholar_r500_wheel_track * angular_speed * scholar_r500_angular_speed_factor) * 10 / (PI * scholar_r500_wheel_diameter * scholar_r500_linear_speed_factor); 
     
     wheel_rate_average = (floor(right_wheel_rate) + ceil(right_wheel_rate)) * 0.5000;
    
     if(right_wheel_rate > wheel_rate_average)motor_speed_temp.value = ceil(right_wheel_rate);
     else motor_speed_temp.value = floor(right_wheel_rate);

     data[5] = motor_speed_temp.u[1];
     data[6] = motor_speed_temp.u[0];


     wheel_rate_average = (floor(left_wheel_rate) + ceil(left_wheel_rate)) * 0.5000;
    
     if(left_wheel_rate > wheel_rate_average)motor_speed_temp.value = ceil(left_wheel_rate);
     else motor_speed_temp.value = floor(left_wheel_rate);

     data[7] = motor_speed_temp.u[1];
     data[8] = motor_speed_temp.u[0];

     write_last_time = write_current_time;

     boost::asio::write(*sp, boost::asio::buffer(&data[0], 10), ec);

}


void scholar_r500_hardware::handle_read( char buffer_data[], boost::system::error_code ec, std::size_t bytes_transferred )
{
 
     read_buffer_size = bytes_transferred;

     if(read_buffer_size == 27)//read 速度指令返回值
     {   
          if ( buffer_data[1] == 0x03 || buffer_data[2] == 0x01 || buffer_data[3] == 0x1A || buffer_data[26] == 0x6D || buffer_data[4] == 0x04)
          {

                   //union motor_speed motor_spped_tem;  

               motor_data.motor_disconnect[0] = buffer_data[7];
               if(!motor_data.motor_disconnect[0])//电机没有失去联系，更新转速
               {
                    motor_data.motor_speed[0].u[1] = buffer_data[5];
                    motor_data.motor_speed[0].u[0] = buffer_data[6];
               }
               else ROS_ERROR("ERROR,   the RIGHT wheel disconnected");

               motor_data.motor_disconnect[1] = buffer_data[10];
               if(!motor_data.motor_disconnect[1])//电机没有失去联系，更新转速
               {
                    motor_data.motor_speed[1].u[1] = buffer_data[8];
                    motor_data.motor_speed[1].u[0] = buffer_data[9];
               }
               else ROS_ERROR("ERROR,   the LEFT wheel disconnected"); 
                 
               sonar[0].u[1] = buffer_data[17];
               sonar[0].u[0] = buffer_data[18];

               sonar[1].u[1] = buffer_data[19];                  
               sonar[1].u[0] = buffer_data[20];


               sonar[2].u[1] = buffer_data[21];
               sonar[2].u[0] = buffer_data[22];

               sonar[3].u[1] = buffer_data[23];                  
               sonar[3].u[0] = buffer_data[24];

               while(read_flag == false)read_flag = true;

          }
          else tcflush(sp->lowest_layer().native_handle(), TCIFLUSH);//清空串口输入

     }
     else if(read_buffer_size == 14)//读取产品信息
     {
          if ( buffer_data[1] == 0x03 || buffer_data[2] == 0x01 || buffer_data[3] == 0x0d || buffer_data[13] == 0x6D || buffer_data[4] == 0x05)
          {
               product_data.product_year.u[1] = buffer_data[5];
               product_data.product_year.u[0] = buffer_data[6];
               product_data.product_month = buffer_data[7];
               product_data.product_day = buffer_data[8];
               product_data.product_number = buffer_data[9];

               while(read_product_data_flag == false)read_product_data_flag = true;

          }
          else tcflush(sp->lowest_layer().native_handle(), TCIFLUSH);//清空串口输入

    }
    else if(read_buffer_size == 8)
    {
       //下发速度指令成功
          if ( buffer_data[1] == 0x03 || buffer_data[2] == 0x01 || buffer_data[3] == 0x07 || buffer_data[13] == 0x6D || buffer_data[4] == 0x03)
          {

               while(write_falg == false)write_falg = true;
               if(buffer_data[5] == 0x05 || buffer_data[6] == 0x05)ROS_WARN_STREAM("Scholar is on SUDDEN stop status");

          }
          else if( buffer_data[1] == 0x03 || buffer_data[2] == 0x01 || buffer_data[3] == 0x07 || buffer_data[13] == 0x6D || buffer_data[4] == 0x07)
          {
               //串口紧急停止返回
               while(sudden_stop_flag == false)sudden_stop_flag = true;
               if(buffer_data[5] == 0x05 || buffer_data[6] == 0x05)ROS_WARN_STREAM("Scholar is on SUDDEN stop status");

          }
          else if( buffer_data[1] == 0x03 || buffer_data[2] == 0x01 || buffer_data[3] == 0x07 || buffer_data[13] == 0x6D || buffer_data[4] == 0x08)
          {
               //声波急停关闭
               while(disable_sonar_sudden_stop_flag == false)disable_sonar_sudden_stop_flag = true;

          }
          else if( buffer_data[1] == 0x03 || buffer_data[2] == 0x01 || buffer_data[3] == 0x07 || buffer_data[13] == 0x6D || buffer_data[4] == 0x09)
          {
               //声波急停打开
               while(enable_sonar_sudden_stop_flag == false)enable_sonar_sudden_stop_flag = true;

          }
          else tcflush(sp->lowest_layer().native_handle(), TCIFLUSH);//清空串口输入


    }
    else tcflush(sp->lowest_layer().native_handle(), TCIFLUSH);//清空串口输入

}

void scholar_r500_hardware::listen_data(int buffer_number, int max_seconds)
{

     boost::asio::deadline_timer timer( iosev); 

     char buf[buffer_number];
            
     iosev.reset();
     boost::asio::async_read(*sp, boost::asio::buffer(buf, sizeof(buf)), boost::bind(&scholar_r500_hardware::handle_read, this, buf, _1,  _2)) ;      
    
     int ti = timer.expires_from_now(boost::posix_time::millisec(max_seconds)) ; //同步等待，程序堵塞     

     timer.wait(ec);
     if(ti > 0)iosev.reset();

     try{

          iosev.run();

     }
     catch(boost::system::system_error& ecc) {
          std::cerr << ecc.what() << std::endl;

     }

}

bool scholar_r500_hardware::read_msg()
{

     while(read_flag == true)read_flag = false;

     try{
          readSpeedCommand(); //向串口发送读转速指令
          listen_data(27,2);
     }
     catch(boost::system::system_error& ecc){

          std::cerr << ecc.what() << std::endl;
          std::cout << "Can't get speed " << std::endl;
 
     }

     current_time = ros::Time::now();

     double read_left_wheel_speed = 0;
     double read_right_wheel_speed = 0;

     double wheel_average_speed = 0;

     vx = PI * scholar_r500_wheel_diameter * scholar_r500_linear_speed_factor * (motor_data.motor_speed[0].value + motor_data.motor_speed[1].value) / 1200.00;//线速度

     vth =  PI * scholar_r500_wheel_diameter * scholar_r500_linear_speed_factor * (motor_data.motor_speed[0].value - motor_data.motor_speed[1].value) / (600 * scholar_r500_angular_speed_factor * scholar_r500_wheel_track);//2021.08.02，角速度

     if(abs(vth)<= 0.05 && abs(vx) < 0.02)
     {
          vth = 0;
          vx = 0;
     }

     double dt = (current_time - last_time).toSec();
     double delta_th = (vth + 2 * vth_old) * dt / 3.00;

     double delta_x = 0.00;
     double delta_y = 0.00;

     delta_x = cos(delta_th * 0.500 + th) * (vx + 2 * vx_old)* dt / 3.00;
     delta_y = sin(delta_th * 0.500 + th) * (vx + 2 * vx_old) * dt / 3.00;

     last_time = current_time;
	vx_old = vx;
	vth_old = vth;

     x += delta_x;
     y += delta_y;
     th += delta_th;

     transformStamped.header.stamp = current_time;
     transformStamped.header.frame_id = "odom";
     transformStamped.child_frame_id = "base_link";
     transformStamped.transform.translation.x = x;
     transformStamped.transform.translation.y = y;
     transformStamped.transform.translation.z = 0.0;

     tf2::Quaternion q;
     q.setRPY(0, 0, th);
     transformStamped.transform.rotation.x = q.x();
     transformStamped.transform.rotation.y = q.y();
     transformStamped.transform.rotation.z = q.z();
     transformStamped.transform.rotation.w = q.w();

     odom.header.frame_id  = "odom";
     odom.child_frame_id = "base_link";
     odom.header.stamp = current_time;
     odom.pose.pose.position.x = x;
     odom.pose.pose.position.y = y;
     odom.pose.pose.position.z = 0;
     odom.pose.pose.orientation.x = q.getX();
     odom.pose.pose.orientation.y = q.getY();
     odom.pose.pose.orientation.z = q.getZ();
     odom.pose.pose.orientation.w = q.getW();
     odom.twist.twist.linear.x = vx;
     odom.twist.twist.linear.y = 0;
     odom.twist.twist.angular.z = vth;

     odom.pose.covariance = odom_pose_covariance;
     odom.twist.covariance = odom_twist_covariance;

     battery.battery_percent = 50;
     battery.battery_voltage = 24.8;

     sonar_range[0].header.frame_id ="sonar_1_link";
     sonar_range[0].header.stamp = current_time;
     sonar_range[0].radiation_type = 0;
     sonar_range[0].field_of_view = 1.0;

     sonar_range[0].min_range = 0.01;
     sonar_range[0].max_range = 0.3;
     sonar_range[0].range = sonar[0].value * 0.001;


     sonar_range[1].header.frame_id ="sonar_2_link";
     sonar_range[1].header.stamp = current_time;
     sonar_range[1].radiation_type = 0;
     sonar_range[1].field_of_view = 1.0;

     sonar_range[1].min_range = 0.01;
     sonar_range[1].max_range = 0.3;
     sonar_range[1].range = sonar[1].value * 0.001;
    
     sonar_range[2].header.frame_id ="sonar_3_link";
     sonar_range[2].header.stamp = current_time;
     sonar_range[2].radiation_type = 0;
     sonar_range[2].field_of_view = 1.0;

     sonar_range[2].min_range = 0.01;
     sonar_range[2].max_range = 0.3;
     sonar_range[2].range = sonar[2].value * 0.001;

     sonar_range[3].header.frame_id ="sonar_4_link";
     sonar_range[3].header.stamp = current_time;
     sonar_range[3].radiation_type = 0;
     sonar_range[3].field_of_view = 1.0;
     sonar_range[3].min_range = 0.01;
     sonar_range[3].max_range = 0.3;
     sonar_range[3].range = sonar[3].value * 0.001;

     return true;

}


bool scholar_r500_hardware::spOnce(double linear_speed, double angular_speed)
{

     while(write_falg == true)write_falg = false;
     try{

          writeSpeedCommand(linear_speed, angular_speed);
          listen_data(8, 2);
     }
     catch(boost::system::system_error& ecc){

          std::cerr << ecc.what() << std::endl;
          std::cout << "Can't send speed " << std::endl;
 
     }
         
     return true;
};


void scholar_r500_hardware::read_product_data()
{

     uint8_t data_read[6] = {0xFF, 0x03, 0x01, 0x06, 0x05, 0x6D};   

     boost::asio::write(*sp, boost::asio::buffer(&data_read[0], 6), ec);

}

void scholar_r500_hardware::sudden_stop()
{

     uint8_t data_read[6] = {0xFF, 0x03, 0x01, 0x06, 0x07, 0x6D};   

     boost::asio::write(*sp, boost::asio::buffer(&data_read[0], 6), ec);

}

void scholar_r500_hardware::disable_sonar_sudden_stop()
{

     uint8_t data_read[6] = {0xFF, 0x03, 0x01, 0x06, 0x08, 0x6D};   

     boost::asio::write(*sp, boost::asio::buffer(&data_read[0], 6), ec);

}

void scholar_r500_hardware::enable_sonar_sudden_stop(uint16_t sonar_sudden_range)
{

     uint8_t data_read[8] = {0xFF, 0x03, 0x01, 0x06, 0x09, 0x00, 0x00, 0x6D};   

     data_read[5] = sonar_sudden_range >> 8 & 0xFF;
     data_read[6] = sonar_sudden_range & 0xFF;


     boost::asio::write(*sp, boost::asio::buffer(&data_read[0], 8), ec);

}


}
