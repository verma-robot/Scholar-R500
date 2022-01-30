#ifndef SCHOLAR_TELEOP_H
#define SCHOLAR_TELEOP_H


#include <termios.h>  
#include <signal.h>  
#include <math.h>  
#include <stdio.h>  
#include <stdlib.h>  
#include <sys/poll.h>  
  
#include <boost/thread/thread.hpp>  
#include <ros/ros.h>  
#include <geometry_msgs/Twist.h>  

#define KEYCODE_W 0x77  
#define KEYCODE_A 0x61  
#define KEYCODE_S 0x73  
#define KEYCODE_D 0x64  

namespace scholar_teleop
{

    class ScholarTeleopNode  
    {    
        private:

            ros::NodeHandle n_;      

            ros::Publisher pub_ ;  


        public:  

            struct termios old_data;
            ScholarTeleopNode(const ros::NodeHandle &nodehandle, std::string new_topic_name);
            ~ScholarTeleopNode() { };  

            void keyboardLoop();           
            void stopRobot(); 
            geometry_msgs::Twist cmdvel_;  

            float vx;
            float vth;
            std::string cmd_vel_new_name;
       
};  



}

#endif /* SCHOLAR_TELEOP_H */
