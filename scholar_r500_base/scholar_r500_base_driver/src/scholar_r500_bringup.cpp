#include "scholar_r500.h"

double linear_speed = 0;
double angular_speed = 0;	

void cmdCallback(const geometry_msgs::Twist& msg)
{

        linear_speed = msg.linear.x ;
        angular_speed = msg.angular.z;
        
}

    
int main(int argc, char** argv)
{
        ros::init(argc,argv,"scholar_r500_bringup", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);  

	ros::NodeHandle nh("~");  

        std::string port_name; 
        std::string ns; 
        int port_rate;
        unsigned int port_rate__;

        double scholar_r500_linear_speed_calibrate_factor;
        double scholar_r500_angular_speed_calibrate_factor;

        float scholar_r500_wheel_diameter;
        float scholar_r500_wheel_track;
        float scholar_r500_max_acc_x;
        float scholar_r500_max_acc_w;
        tf2_ros::TransformBroadcaster broadcast_tf; 

        nh.getParam("port_name", port_name);
        nh.getParam("port_rate", port_rate);
        nh.getParam("scholar_r500_linear_speed_calibrate_factor", scholar_r500_linear_speed_calibrate_factor);
        nh.getParam("scholar_r500_angular_speed_calibrate_factor", scholar_r500_angular_speed_calibrate_factor);
        nh.getParam("scholar_r500_wheel_diameter", scholar_r500_wheel_diameter);
        nh.getParam("scholar_r500_wheel_track", scholar_r500_wheel_track);
        nh.getParam("scholar_r500_max_acc_x", scholar_r500_max_acc_x);
        nh.getParam("scholar_r500_max_acc_w", scholar_r500_max_acc_w);

        nh.getParam("ns", ns);

        port_rate__ = (unsigned int)port_rate;

        scholar_r500::scholar_r500_hardware robot(port_name, port_rate);

	if(robot.scholar_r500_ready == true)
        { 
                ROS_INFO("Scholar R500 initialized successful.");
           
                std::cout << "The Data of Production :" << robot.product_data.product_year.value <<  " year; "<<(int16_t)robot.product_data.product_month << " month, " <<  (int16_t)robot.product_data.product_day << " day. "<< (int16_t)robot.product_data.product_number << std::endl;

                std::cout << "The Serial Number :" <<  (int16_t)robot.product_data.product_number << std::endl;

                robot.scholar_r500_linear_speed_factor = scholar_r500_linear_speed_calibrate_factor;
                robot.scholar_r500_angular_speed_factor = scholar_r500_angular_speed_calibrate_factor;
                robot.scholar_r500_wheel_diameter = scholar_r500_wheel_diameter;
                robot.scholar_r500_wheel_track = scholar_r500_wheel_track;
                robot.scholar_r500_max_acc_x = scholar_r500_max_acc_x;
                robot.scholar_r500_max_acc_w = scholar_r500_max_acc_w;

        }
	else if(!robot.scholar_r500_ready)ROS_ERROR("Scholar Robot initialized failed.");
    
	ros::Subscriber sub = nh.subscribe("/cmd_vel", 10, &cmdCallback);
        ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>(ns + "/odom", 10);

        ros::Publisher battery_pub = nh.advertise<scholar_r500_base_driver::scholar_r500_battery>(ns + "/battery", 10);    
        ros::Publisher sonar_1_pub = nh.advertise<sensor_msgs::Range>(ns + "/sonar_1_range", 10);
        ros::Publisher sonar_2_pub = nh.advertise<sensor_msgs::Range>(ns + "/sonar_2_range", 10);
        ros::Publisher sonar_3_pub = nh.advertise<sensor_msgs::Range>(ns + "/sonar_3_range", 10);
        ros::Publisher sonar_4_pub = nh.advertise<sensor_msgs::Range>(ns + "/sonar_4_range", 10);

	ros::Rate loop_rate(10);
	while (ros::ok()) 
	{
                try
                {
                        if(robot.read_msg() == true)
                        {
                                odom_pub.publish(robot.odom);
                                battery_pub.publish(robot.battery);
                                broadcast_tf.sendTransform(robot.transformStamped);
                                sonar_1_pub.publish(robot.sonar_range[0]);
                                sonar_2_pub.publish(robot.sonar_range[1]);
                                sonar_3_pub.publish(robot.sonar_range[2]);
                                sonar_4_pub.publish(robot.sonar_range[3]);
                                while(robot.read_flag == true)robot.read_flag = false;
                        }
                
                        robot.spOnce(linear_speed, angular_speed);

                        while(robot.write_falg == true){robot.write_falg = false;};
       
	                ros::spinOnce();
	                loop_rate.sleep();
                }
                catch(const std::exception& e)
                {

		        ROS_ERROR("//////////////ERROR/////////////.");

                }
	}

	return 0;
}