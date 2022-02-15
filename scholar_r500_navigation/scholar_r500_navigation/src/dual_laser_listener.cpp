#include "dual_laser_listener.h"

namespace scholar_dual_laser
{

    ScholarDualLaserNode::ScholarDualLaserNode(const ros::NodeHandle &nodehandle):n_(nodehandle)
    {

        front_laser_sub = n_.subscribe<sensor_msgs::LaserScan>("front_scan", 1, &ScholarDualLaserNode::front_laser_callback , this);  
        rear_laser_sub = n_.subscribe<sensor_msgs::LaserScan>("rear_scan", 1, &ScholarDualLaserNode::rear_laser_callback , this);
        odom_filter_sub = n_.subscribe<nav_msgs::Odometry>("/odometry/filtered", 1, &ScholarDualLaserNode::odom_filter_callback,this);
        pub_ = n_.advertise<sensor_msgs::PointCloud2>("rear_laser_to_point", 1); 

        get_front_laser = false;
        get_rear_laser = false;

        total_cloud.resize(3300);

        for(int i = 0 ; i < total_cloud.size() ; i++) 
        {
            total_cloud.points[i].x = 0.00;
            total_cloud.points[i].y = 0.00;
            total_cloud.points[i].z = 0.20;

        }       
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
                    total_cloud.points[i].x = 0;
                    total_cloud.points[i].y = 0;
                    total_cloud.points[i].z = 0.2;

                }       
                for(int i = 0; i < front_cloud.points.size() ; i ++)
                {
                    total_cloud.points[i].x = front_cloud.points[i].x;
                    total_cloud.points[i].data[1] = front_cloud.points[i].data[1];
                    total_cloud.points[i].data[2] = front_cloud.points[i].data[2];
                    total_cloud.points[i].data[3] = front_cloud.points[i].data[3];
                }
            }

            get_front_laser = true;

            ros::Time now_time = ros::Time::now();
            float delt_time =  (now_time - rear_last_time ).toSec() ;
            delt_time = abs(delt_time);

            float delt_oula =   (w_ + 2 * old_w_) * delt_time / 3;
            float delt_distance_x = (vel * delt_time * cos(delt_oula * 0.5));
            float delt_distance_y = (vel * delt_time * sin(delt_oula * 0.5 ));


            int liqun_number = 0;
            int inqun_number = 0;
            bool sartnewqun = false;
            int newqun_start_number = 0;

            for(int i = 0 ; i < total_cloud.size() ; i++)
            {
                if(total_cloud.points[i].x != 0.00 && total_cloud.points[i].y != 0.00)
                {
                    total_cloud.points[i].x -= delt_distance_x;
                    total_cloud.points[i].y -= delt_distance_y;
                    
                    float a = atan(total_cloud.points[i].y / total_cloud.points[i].x);
                    a -= (delt_oula * 1.0);                        
                    float distance = sqrt(total_cloud.points[i].x * total_cloud.points[i].x + total_cloud.points[i].y * total_cloud.points[i].y);
                    if(distance > 0.3)
                    {
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
                    else              
                    {
                        total_cloud.points[i].x = 0;
                        total_cloud.points[i].y = 0;  
                    }
                }
                float max_theta =  atan2( front_laser.range_max *cos(filter_angular), front_laser.range_max * sin(filter_angular) + 0.2);

                float theta_ = 0.00;
                if(fabs(total_cloud.points[i].x) > 0)theta_ = atan2(fabs(total_cloud.points[i].y) , fabs(total_cloud.points[i].x));
                else theta_ = 1.57;
                if(theta_ > max_theta) 
                {
                    total_cloud.points[i].x = 0;
                    total_cloud.points[i].y = 0;  
                }

                if(i == 0 && total_cloud.points[0].x != 0 )
                {
                    inqun_number  = 1;
                    newqun_start_number = 0;
                    sartnewqun = false;

                }
                else if( i ==0 &&  total_cloud.points[0].x == 0 )
                {
                    liqun_number = 1;
                    sartnewqun = false;
                }
                else if(i > 0 && total_cloud.points[0].x != 0 )
                {
                    inqun_number++;
                    if(sartnewqun == true && inqun_number  == 0)newqun_start_number = i;
                    sartnewqun = false;
                }
                else if(i > 0 && total_cloud.points[0].x == 0 )
                {
                    liqun_number++;
                    sartnewqun = false;
                }

                if(inqun_number >= 20 && liqun_number < 10)
                {
                    liqun_number = 0;
                    inqun_number = 0;
                    sartnewqun = true;
                    newqun_start_number = i;
                }
                else if(liqun_number > 10 && inqun_number < 20)
                {
                    for(int j = newqun_start_number; j < i; j++)
                    {
                        total_cloud.points[j].x = 0;
                        total_cloud.points[j].y = 0;                             
                    }
                    liqun_number = 0;
                    inqun_number = 0;
                    sartnewqun = true;
                    newqun_start_number = i;
                }                
            }

            pcl::toROSMsg(total_cloud , dual_laser_cloud);
            dual_laser_cloud.header.stamp = ros::Time::now();
            dual_laser_cloud.header.frame_id = "laser_fix_link";
            pub_.publish(dual_laser_cloud);
            front_last_time = now_time;
            done_together = true;        
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
                    total_cloud.points[i].x = 0;
                    total_cloud.points[i].y = 0;
                    total_cloud.points[i].z = 0.2;

                }          
                for(int i = 0; i < rear_cloud.points.size() ; i ++)total_cloud.points[i + 1650] = rear_cloud.points[i];
            }
            get_rear_laser = true;


            ros::Time now_time = ros::Time::now();
            float delt_time =  (now_time - front_last_time ).toSec() ;
            delt_time = abs(delt_time);

            float delt_oula =   (w_ + 2 * old_w_) * delt_time / 3;
            float delt_distance_x = (vel * delt_time * cos(delt_oula * 0.5));
            float delt_distance_y = (vel * delt_time * sin(delt_oula * 0.5 ));

            int liqun_number = 0;
            int inqun_number = 0;
            bool sartnewqun = false;
            int newqun_start_number = 0;


            for(int i = 0 ; i < total_cloud.size() ; i++)
            {
                if(total_cloud.points[i].x != 0.00 && total_cloud.points[i].y != 0.00)
                {
                    total_cloud.points[i].x -= delt_distance_x;
                    total_cloud.points[i].y -= delt_distance_y;

                    
                    float a = atan(total_cloud.points[i].y / total_cloud.points[i].x);
                    a -= (delt_oula * 1.0);
                    float distance = sqrt(total_cloud.points[i].x * total_cloud.points[i].x + total_cloud.points[i].y * total_cloud.points[i].y);
                    if(distance > 0.3)
                    {
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
                    else//前向雷达，点云位于前方，删除雷达平行及后方的点
                    {
                        total_cloud.points[i].x = 0;
                        total_cloud.points[i].y = 0;
                    }  
                }


                float max_theta =  atan2( rear_laser.range_max *cos(filter_angular), rear_laser.range_max * sin(filter_angular) + 0.2);


                float theta_ = 0.00;
                if(fabs(total_cloud.points[i].x) > 0)theta_ = atan2(fabs(total_cloud.points[i].y) , fabs(total_cloud.points[i].x));
                else theta_ = 1.57;
                if(theta_ > max_theta) 
                {
                    total_cloud.points[i].x = 0;
                    total_cloud.points[i].y = 0;  
                }

                if(i == 0 && total_cloud.points[0].x != 0 )
                {
                    inqun_number  = 1;
                    newqun_start_number = 0;
                    sartnewqun = false;

                }
                else if( i ==0 &&  total_cloud.points[0].x == 0 )
                {
                    liqun_number = 1;
                    sartnewqun = false;
                }
                else if(i > 0 && total_cloud.points[0].x != 0 )
                {
                    inqun_number++;
                    if(sartnewqun == true && inqun_number  == 0)newqun_start_number = i;
                    sartnewqun = false;
                }
                else if(i > 0 && total_cloud.points[0].x == 0 )
                {
                    liqun_number++;
                    sartnewqun = false;
                }

                if(inqun_number >= 20 && liqun_number < 10)
                {
                    liqun_number = 0;
                    inqun_number = 0;
                    sartnewqun = true;
                    newqun_start_number = i;
                }
                else if(liqun_number > 10 && inqun_number < 20)
                {
                    for(int j = newqun_start_number; j < i; j++)
                    {
                        total_cloud.points[j].x = 0;
                        total_cloud.points[j].y = 0;                             
                    }
                    liqun_number = 0;
                    inqun_number = 0;
                    sartnewqun = true;
                    newqun_start_number = i;
                }
                       //             total_cloud.points[i].z = 0.2;

            }
            
           
            pcl::toROSMsg(total_cloud , dual_laser_cloud);
            dual_laser_cloud.header.stamp = ros::Time::now();
            dual_laser_cloud.header.frame_id = "laser_fix_link";
            pub_.publish(dual_laser_cloud);
            rear_last_time =now_time;
            done_together = true;
        }    
      
    };
    void ScholarDualLaserNode::odom_filter_callback(const nav_msgs::OdometryConstPtr& msg)
    {

        nav_msgs::Odometry odom_msg;
        odom_msg = *msg;

        vel = odom_msg.twist.twist.linear.x;
        w_ = odom_msg.twist.twist.angular.z;
    };
/*
    void ScholarDualLaserNode::joint_together()
    {

        if(get_front_laser || get_rear_laser )
        {

            now_time = ros::Time::now();
            float delt_time =   (now_time - last_time).toSec() ;

            float delt_oula =   (w_ + 2 * old_w_) * delt_time / 3;
            float delt_distance_x = (vel * delt_time * cos(delt_oula));
            float delt_distance_y = (vel * delt_time * sin(delt_oula ));

            for(int i = 0 ; i < total_cloud.size() ; i++)
            {
                if(total_cloud.points[i].x != 0.00 && total_cloud.points[i].y != 0.00)
                {
                    total_cloud.points[i].x -= delt_distance_x;
                    total_cloud.points[i].y -= delt_distance_y;
              
                }
            }

            pcl::toROSMsg(total_cloud , dual_laser_cloud);
            dual_laser_cloud.header.stamp = ros::Time::now();
            dual_laser_cloud.header.frame_id = "base_link";
            pub_.publish(dual_laser_cloud);
            done_together = true;
 
        }
        else done_together = false;
        //last_time = now_time;
        old_w_ = w_;

    };
*/
};


int main(int argc, char** argv)  
{  
    ros::init(argc,argv,"dual_laser_sub", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);  
    
    ros::NodeHandle nh_; 

    scholar_dual_laser::ScholarDualLaserNode joint_together(nh_);

/*
	ros::Rate loop_rate(20);
         
	while (ros::ok()) 
	{

        try
        {

            joint_together.joint_together();
            if(joint_together.done_together == true)
            {
                pub_.publish(joint_together.dual_laser_cloud);
                //joint_together.total_cloud.resize(3300);
              //  joint_together.done_together = false;
            }
	        ros::spinOnce();
	        loop_rate.sleep();
        }
        catch(const std::exception& e)
        {

		    ROS_ERROR("//////////////ERROR/////////////.");

        }
	}
*/
	ros::spin();

    return(0);  
}  

