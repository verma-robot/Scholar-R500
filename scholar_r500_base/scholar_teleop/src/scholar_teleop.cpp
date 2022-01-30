#include "scholar_teleop.h"
 

namespace scholar_teleop
{

    ScholarTeleopNode::ScholarTeleopNode(const ros::NodeHandle &nodehandle, std::string new_topic_name):
         n_(nodehandle)
    {

        cmdvel_.linear.x = 0.00;
        cmdvel_.linear.y = 0.00;
        cmdvel_.linear.z = 0.00;
        cmdvel_.angular.x = 0.00;  
        cmdvel_.angular.y = 0.00;
        cmdvel_.angular.z = 0.00;  
        pub_ = n_.advertise<geometry_msgs::Twist>(new_topic_name, 1);  

    };


    void ScholarTeleopNode::keyboardLoop()  
    {  
        char number_get;  
        bool release = false;  

        struct termios new_data; 

        tcgetattr(0, &old_data);  

        memcpy(&new_data, &old_data, sizeof(struct termios));  

        new_data.c_lflag &= ~(ICANON | ECHO);  
        new_data.c_cc[VEOL] = 1;  
        new_data.c_cc[VEOF] = 2;  
        tcsetattr(0, TCSANOW, &new_data);  
      
        puts("Reading from keyboard");  
        puts("Use WASD keys to control the robot");  
        puts("W : Move front");  
        puts("S : Move back");  
        puts("A : Turn left ");  
        puts("D : Turn right ");  

      
        struct pollfd ufd;  
        ufd.fd = 0;  
        ufd.events = POLLIN;  
      
        for(;;)  
        {  
            boost::this_thread::interruption_point();  
          
            int num;  
          
            if ((num = poll(&ufd, 1, 500)) < 0)  
            {  
                 perror("poll():");  
                return;  
            }  
            else if(num > 0)  
            {  
                if(read(0, &number_get, 1) < 0) //read one 
                {  
                    usleep(200);
                    perror("read():"); 
                    return;  
                }  
            }  
            else  
            {   
                if (release == true)  
                {  
                    stopRobot();  
                    release = false;  
                } 
                               
                continue;  
            }  
          
            switch(number_get)  
            {  
                case KEYCODE_W:  
                    cmdvel_.linear.x = vx;  
                    cmdvel_.angular.z = 0;  
                    pub_.publish(cmdvel_);
                    release = true;  
                    break;  
                case KEYCODE_S:  
                    cmdvel_.linear.x = -vx;  
                    cmdvel_.angular.z = 0; 
                    pub_.publish(cmdvel_); 
                    release = true;  
                    break;  
                 case KEYCODE_A:  
                    cmdvel_.linear.x = 0;  
                    cmdvel_.angular.z =vth; 
                    pub_.publish(cmdvel_); 
                    release = true;  
                    break;  
                case KEYCODE_D:  
                    cmdvel_.linear.x = 0;  
                    cmdvel_.angular.z = -vth; 
                    pub_.publish(cmdvel_); 
                    release = true;  
                    break;  
               
                default:
                   // cmdvel_.linear.x = 0;  
                   // cmdvel_.angular.z = 0;  
                    release = false;  
                    break;
            }  
            
            
        } 


    };

    void ScholarTeleopNode::stopRobot()
    {

        cmdvel_.linear.x = 0.0;  
        cmdvel_.angular.z = 0.0;  
        pub_.publish(cmdvel_);  

    };  
}