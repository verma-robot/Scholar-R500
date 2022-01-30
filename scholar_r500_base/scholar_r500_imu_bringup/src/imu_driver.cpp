#include "imu.h"
    
int main(int argc, char** argv)
{

	ros::init(argc, argv, "imu_driver");

	ros::NodeHandle nh("~");

        std::string port_name; 
        int port_rate;
        float g;
        std::string ns;

        nh.getParam("port_name", port_name);
        nh.getParam("port_rate", port_rate);
        nh.getParam("g", g);

        nh.getParam("ns",ns);

        scholar_r500_imu::IMU_DRIVER IMU(port_name, port_rate, g);

        if(IMU.IMU_READY)ROS_INFO("imu initialized successful.");
        else if(IMU.IMU_READY == false)ROS_ERROR("imu initialized failed.");

        ros::Publisher pub = nh.advertise<sensor_msgs::Imu>(ns + "/imu_data", 50);

	ros::Rate loop_rate(100);
	while (ros::ok()) 
	{
                try
                {
                        if(IMU.IMU_READY)
                        {
                                IMU.read_msg();
                                pub.publish(IMU.imu_data);
                        }
 
                        loop_rate.sleep();
                }
                catch(const std::exception& e)
                {
		        ROS_ERROR("//////////////ERROR/////////////.");

                }
	}

	return 0;
}


