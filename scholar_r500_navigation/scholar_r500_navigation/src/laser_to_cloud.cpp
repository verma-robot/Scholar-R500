
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <math.h>
#include <boost/thread/thread.hpp>  

#include <termios.h>  
#include <signal.h>  
#include <stdio.h>  
#include <stdlib.h>  
#include <sys/poll.h>  
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>

ros::Publisher pub_;
laser_geometry::LaserProjection projector_;

void  rear_laser_callback(const sensor_msgs::LaserScanConstPtr& msg)
{
    sensor_msgs::LaserScan rear_laser;
    rear_laser = *msg;
    int length_rear = rear_laser.ranges.size();

    if(length_rear != 0)
    {
        sensor_msgs::PointCloud2 cloud;
        projector_.projectLaser(rear_laser, cloud);
        pub_.publish(cloud);

    }    
};

int main(int argc, char** argv)  
{  
    ros::init(argc,argv,"dual_laser_sub", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);  
    
    ros::NodeHandle nh_; 

    ros::Subscriber  rear_laser_sub = nh_.subscribe<sensor_msgs::LaserScan>("rear_scan", 1, rear_laser_callback );

    pub_ = nh_.advertise<sensor_msgs::PointCloud2>("rear_laser_to_point", 1); 


	ros::spin();

    return(0);  
}  

