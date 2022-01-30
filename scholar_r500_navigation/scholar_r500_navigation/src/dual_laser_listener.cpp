#include "dual_laser_listener.h"

namespace scholar_dual_laser
{

    ScholarDualLaserNode::ScholarDualLaserNode(const ros::NodeHandle &nodehandle):n_(nodehandle)
    {

        front_laser_sub = n_.subscribe<sensor_msgs::LaserScan>("front_scan", 1, &ScholarDualLaserNode::front_laser_callback , this);  
        rear_laser_sub = n_.subscribe<sensor_msgs::LaserScan>("rear_scan", 1, &ScholarDualLaserNode::rear_laser_callback , this);
        odom_filter_sub = n_.subscribe<nav_msgs::Odometry>("/odometry/filtered", 1, &ScholarDualLaserNode::odom_filter_callback,this);
        pub_ = n_.advertise<sensor_msgs::PointCloud2>("rear_laser_to_point", 1); 

        total_cloud.resize(3300);
        front_last_time = ros::Time::now();
        rear_last_time = ros::Time::now();


    }

    void ScholarDualLaserNode::front_laser_callback(const sensor_msgs::LaserScanConstPtr& msg)
   {
        sensor_msgs::LaserScan front_laser;
        front_laser = *msg;
        int length_front = front_laser.ranges.size();

        if(length_front != 0)
        {   
            sensor_msgs::PointCloud2 cloud;
            projector_.transformLaserScanToPointCloud("laser_fix_link" , front_laser, cloud, tf_listner);

            pcl::fromROSMsg(cloud, front_cloud);

            if(front_cloud.points.size()!=0)
            {

                for(int i = 0 ; i < 1650 ; i++) 
                {
                    total_cloud.points[i].x = 0.00;
                    total_cloud.points[i].y = 0.00;
                    total_cloud.points[i].z = 0.00;

                }       
                for(int i = 0; i < front_cloud.points.size() ; i ++)
                {
                    total_cloud.points[i].data[0] = front_cloud.points[i].data[0];
                    total_cloud.points[i].data[1] = front_cloud.points[i].data[1];
                    total_cloud.points[i].data[2] = front_cloud.points[i].data[2];
                    total_cloud.points[i].data[3] = front_cloud.points[i].data[3];
                }
            }

            ros::Time now_time = ros::Time::now();
            float delt_time =  (now_time - rear_last_time ).toSec() ;
            delt_time = abs(delt_time);

            float delt_oula =   (w_ + 2 * old_w_) * delt_time / 3;
            float delt_distance_x = (vel * delt_time * cos(delt_oula * 0.5));
            float delt_distance_y = (vel * delt_time * sin(delt_oula * 0.5 ));

            for(int i = 0 ; i < total_cloud.size() ; i++)
            {
                if(total_cloud.points[i].x != 0.00 && total_cloud.points[i].y != 0.00)
                {
                    total_cloud.points[i].x -= delt_distance_x;
                    total_cloud.points[i].y -= delt_distance_y;

                    if(abs(total_cloud.points[i].x) > 0.05)//前向雷达，点云位于前方，删除雷达平行及后方的点
                    {
                        float a = atan(total_cloud.points[i].y / total_cloud.points[i].x);
                        a -= (delt_oula * 1.0);                        
                        float distance = sqrt(total_cloud.points[i].x * total_cloud.points[i].x + total_cloud.points[i].y * total_cloud.points[i].y);
                        if(i < 1650)
                        {
                            total_cloud.points[i].x = +distance * cos(a);
                             total_cloud.points[i].y = +distance * sin(a);  
                        }
                        else
                        {
                             total_cloud.points[i].x = -distance * cos(a);
                             total_cloud.points[i].y = -distance * sin(a);  

                        }
                    }
              
                }
            }
            pcl::toROSMsg(total_cloud , dual_laser_cloud);
            dual_laser_cloud.header.stamp = ros::Time::now();
            dual_laser_cloud.header.frame_id = "laser_fix_link";
            pub_.publish(dual_laser_cloud);
            front_last_time = now_time;
	}    
   };


   void  ScholarDualLaserNode::rear_laser_callback(const sensor_msgs::LaserScanConstPtr& msg)
   {
        sensor_msgs::LaserScan rear_laser;
        rear_laser = *msg;
        int length_rear = rear_laser.ranges.size();

        if(length_rear != 0)
        {
            sensor_msgs::PointCloud2 cloud;
            projector_.transformLaserScanToPointCloud("laser_fix_link" , rear_laser, cloud, tf_listner);
            pcl::fromROSMsg(cloud, rear_cloud);

            if(rear_cloud.points.size()!=0)
            {    
                for(int i = 1650 ; i < 3300 ; i++) 
                {
                    total_cloud.points[i].x = 0.00;
                    total_cloud.points[i].y = 0.00;
                    total_cloud.points[i].z = 0.00;

                }          
                for(int i = 0; i < rear_cloud.points.size() ; i ++)total_cloud.points[i + 1650] = rear_cloud.points[i];
            }

            ros::Time now_time = ros::Time::now();
            float delt_time =  (now_time - front_last_time ).toSec() ;
            delt_time = abs(delt_time);

            float delt_oula =   (w_ + 2 * old_w_) * delt_time / 3;
            float delt_distance_x = (vel * delt_time * cos(delt_oula * 0.5));
            float delt_distance_y = (vel * delt_time * sin(delt_oula * 0.5 ));

            for(int i = 0 ; i < total_cloud.size() ; i++)
            {
                if(total_cloud.points[i].x != 0.00 && total_cloud.points[i].y != 0.00)
                {
                    total_cloud.points[i].x -= delt_distance_x;
                    total_cloud.points[i].y -= delt_distance_y;

                    if(abs(total_cloud.points[i].x) > 0.05)//前向雷达，点云位于前方，删除雷达平行及后方的点
                    {
                        float a = atan(total_cloud.points[i].y / total_cloud.points[i].x);
                        a -= (delt_oula * 1.0);
                        float distance = sqrt(total_cloud.points[i].x * total_cloud.points[i].x + total_cloud.points[i].y * total_cloud.points[i].y);
                        
                        if(i < 1650)
                        {
                            total_cloud.points[i].x = distance * cos(a);
                            total_cloud.points[i].y = distance * sin(a);  
                        }
                        else
                        {
                            total_cloud.points[i].x = -distance * cos(a);
                            total_cloud.points[i].y = -distance * sin(a);  
                        }
                    }
              
                }
            }

            pcl::toROSMsg(total_cloud , dual_laser_cloud);
            dual_laser_cloud.header.stamp = ros::Time::now();
            dual_laser_cloud.header.frame_id = "laser_fix_link";
            pub_.publish(dual_laser_cloud);
            rear_last_time = now_time;
        }    
    };
    void ScholarDualLaserNode::odom_filter_callback(const nav_msgs::OdometryConstPtr& msg)
    {

        nav_msgs::Odometry odom_msg;
        odom_msg = *msg;

        vel = odom_msg.twist.twist.linear.x;
        w_ = odom_msg.twist.twist.angular.z;
    };

};


int main(int argc, char** argv)  
{  
    ros::init(argc,argv,"dual_laser_sub", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);  
    
    ros::NodeHandle nh_; 

    scholar_dual_laser::ScholarDualLaserNode joint_together(nh_);

    ros::spin();

    return(0);  
}  

