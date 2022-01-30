#include <vector>
#include "imu.h"

namespace scholar_r500_imu
{

boost::array<double, 9> vel_covariance = {
    {4.44e-5, 0, 0, 
    0, 5.87e-5, 0, 
    0, 0, 2e-7
    }};

boost::array<double, 9> ang_covariance = {
    {2.54e-7, 0, 0, 
    0, 5.97e-7, 0, 
    0, 0, 0.003
    }};

boost::array<double, 9> pose_covariance = {
    {0.1, 0, 0, 
    0, 0.1, 0, 
    0, 0, 0.1
    }};

IMU_DRIVER::IMU_DRIVER(std::string port_name, unsigned int port_rate , float g):sp(NULL)
{ 
    sp = new boost::asio::serial_port(iosev);
    if(sp)init(port_name, port_rate, g);
}

IMU_DRIVER::~IMU_DRIVER()
{
    if(sp)delete sp;

    if (sp) {
        sp -> cancel();
        sp -> close();

    }

    iosev.stop();
    iosev.reset();

    delete sp;

}

bool IMU_DRIVER::init(std::string port_name, int port_rate, float g)
{

    try 
    {

        sp -> open(port_name, ec); 
        if(ec)
        {       
            ROS_ERROR( "Can't open serial port") ;
            throw ec;
        }
        usleep(20000); 

        tcflush(sp->lowest_layer().native_handle(), TCIOFLUSH);//清空串口输入输出缓存

        sp -> set_option(boost::asio::serial_port::baud_rate(port_rate));
	    sp -> set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
	    sp -> set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
	    sp -> set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
	    sp -> set_option(boost::asio::serial_port::character_size(8));

        acc_g = g;
        ros::Time::init();
	    current_time = ros::Time::now();
	    last_time = ros::Time::now();
        IMU_READY = true;
        return true;

    }
    catch(...) {

        IMU_READY = false;
        ROS_ERROR( "Can't open serial port") ;
        return false;
    }       

}

bool IMU_DRIVER::read_msg()
{

    uint8_t buffer_data[182], serial_data[91];

    int length;

    try{

        length=boost::asio::read(*sp, boost::asio::buffer(buffer_data), ec);
        //std::cout << length << std::endl;
        if(length >= 182)
        {

           // if(buffer_data[0] == 0x41 && buffer_data[0 + 1] == 0x78 && buffer_data[0 + 3] == 0x06 && buffer_data[0 + 4] == 0x81
             //      && buffer_data[0 + 5] == 0x53 && buffer_data[0 + 90] == 0x6D)std::cout << "ok"<<std::endl;
            //       else std::cout << "error" <<std::endl;
//std::cout << (int16_t)buffer_data[0] << std::endl;


            for(int i = 0; i < 91; i ++)
            {

                if(buffer_data[i] == 0x41 && buffer_data[i + 1] == 0x78 && buffer_data[i + 3] == 0x06 && buffer_data[i + 4] == 0x81
                   && buffer_data[i + 5] == 0x53 && buffer_data[i + 90] == 0x6D)
                {

                    for(int j = 0; j < 91; j ++)
                    {
                        serial_data[j] = buffer_data[i + j];
                    }
                                  
                }
            }

            if(serial_data[6] == 0x01 && serial_data[7] == 0x88 && serial_data[8] == 0x0C)
            {
                 
                temp_data.u[0] = serial_data[9];
                temp_data.u[1] = serial_data[10];
                temp_data.u[2] = serial_data[11];
                temp_data.u[3] = serial_data[12];
                imu_data.linear_acceleration.x = temp_data.f * acc_g;

                temp_data.u[0] = serial_data[13];
                temp_data.u[1] = serial_data[14];
                temp_data.u[2] = serial_data[15];
                temp_data.u[3] = serial_data[16];
                imu_data.linear_acceleration.y = temp_data.f * acc_g;

                temp_data.u[0] = serial_data[17];
                temp_data.u[1] = serial_data[18];
                temp_data.u[2] = serial_data[19];
                temp_data.u[3] = serial_data[20];
                imu_data.linear_acceleration.z = temp_data.f * acc_g;

            }

            if(serial_data[21] == 0x00 && serial_data[22] == 0x8C && serial_data[23] == 0x0C)
            {
                 
                temp_data.u[0] = serial_data[24];
                temp_data.u[1] = serial_data[25];
                temp_data.u[2] = serial_data[26];
                temp_data.u[3] = serial_data[27];
                imu_data.angular_velocity.x = temp_data.f * PI / 180.00;

                temp_data.u[0] = serial_data[28];
                temp_data.u[1] = serial_data[29];
                temp_data.u[2] = serial_data[30];
                temp_data.u[3] = serial_data[31];
                imu_data.angular_velocity.y = temp_data.f * PI / 180.00;

                temp_data.u[0] = serial_data[32];
                temp_data.u[1] = serial_data[33];
                temp_data.u[2] = serial_data[34];
                temp_data.u[3] = serial_data[35];
                imu_data.angular_velocity.z = temp_data.f * PI / 180.00;


              }
            if(serial_data[51] == 0x00 && serial_data[52] == 0xB0 && serial_data[53] == 0x10)
            {
                 
                temp_data.u[0] = serial_data[54];
                temp_data.u[1] = serial_data[55];
                temp_data.u[2] = serial_data[56];
                temp_data.u[3] = serial_data[57];
                imu_data.orientation.w = temp_data.f;

                temp_data.u[0] = serial_data[58];
                temp_data.u[1] = serial_data[59];
                temp_data.u[2] = serial_data[60];
                temp_data.u[3] = serial_data[61];
                imu_data.orientation.x = temp_data.f;

                temp_data.u[0] = serial_data[62];
                temp_data.u[1] = serial_data[63];
                temp_data.u[2] = serial_data[64];
                temp_data.u[3] = serial_data[65];
                imu_data.orientation.y = temp_data.f;


                temp_data.u[0] = serial_data[66];
                temp_data.u[1] = serial_data[67];
                temp_data.u[2] = serial_data[68];
                temp_data.u[3] = serial_data[69];
                imu_data.orientation.z = temp_data.f;

            }           
            imu_data.header.stamp = ros::Time::now();
            imu_data.header.frame_id = "imu_link";
            imu_data.linear_acceleration_covariance = vel_covariance;
            imu_data.angular_velocity_covariance = ang_covariance;
            imu_data.orientation_covariance = pose_covariance;  
  
            return true;

    }   
       
    else 
        {

            throw exception;
        } 
    }
    catch(bool  exception)
    {
        ROS_ERROR("SCHOLAR IMU ERROR DATA");

        return false;

    } 
    tcflush(sp->lowest_layer().native_handle(), TCIFLUSH);//清空串口输入输出缓存
    

}

}
