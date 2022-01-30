#ifndef SCHOLAR_LASER_H
#define SCHOLAR_LASER_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <math.h>
#include <boost/thread/thread.hpp>  

#include <termios.h>  
#include <signal.h>  
#include <stdio.h>  
#include <stdlib.h>  
#include <sys/poll.h>  
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include<pcl/visualization/pcl_visualizer.h>

namespace scholar_dual_laser
{

    class ScholarDualLaserNode  
    {    
        private:

            ros::NodeHandle n_;      
            ros::Subscriber front_laser_sub;
            ros::Subscriber rear_laser_sub;
            ros::Subscriber odom_filter_sub;
            ros::Publisher pub_;

        public:  

            struct termios old_data;
            ScholarDualLaserNode(const ros::NodeHandle &nodehandle);
            ~ScholarDualLaserNode() { };  

            void front_laser_callback(const sensor_msgs::LaserScanConstPtr& msg);
            void rear_laser_callback(const sensor_msgs::LaserScanConstPtr& msg);
            void odom_filter_callback(const nav_msgs::OdometryConstPtr& msg);

            laser_geometry::LaserProjection projector_;

            tf::TransformListener tf_listner;

            pcl::PointCloud<pcl::PointXYZ> total_cloud;
            
            pcl::PointCloud<pcl::PointXYZ> front_cloud;
            pcl::PointCloud<pcl::PointXYZ> rear_cloud;

            sensor_msgs::PointCloud2 dual_laser_cloud;            

            ros::Time front_last_time;
            ros::Time rear_last_time;

            float vel = 0.00;
            float w_ = 0.00;

            float old_w_ = 0.00;
            float old_vel = 0.00;


    };  
}

#endif 
