#include "scholar_teleop.h"

struct termios original_data;

void handler(int sig)
{

    if(sig == SIGINT)
    {
        tcsetattr(0, TCSANOW, &original_data); 
        std::cout << "okok" <<std::endl;
        exit(0);
    }

};

int main(int argc, char** argv)  
{  
    ros::init(argc,argv,"scholar_teleop", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);      

    ros::NodeHandle n_("~");      

    float scholar_teleop_vx_speed;
    float scholar_teleop_angular_speed ;
    std::string  cmd_vel_topic_name;
    
    n_.getParam("scholar_teleop_vx_speed", scholar_teleop_vx_speed);
    n_.getParam("scholar_teleop_angular_speed", scholar_teleop_angular_speed);
    n_.getParam("cmd_vel_topic_name", cmd_vel_topic_name);
    
    scholar_teleop::ScholarTeleopNode scholar_teleop_data(n_ , cmd_vel_topic_name);  

    scholar_teleop_data.vx = (float)scholar_teleop_vx_speed;
    scholar_teleop_data.vth = scholar_teleop_angular_speed;

    tcgetattr(0, &scholar_teleop_data.old_data);  

    original_data = scholar_teleop_data.old_data;

    signal(SIGINT, handler);

    boost::thread mover(boost::bind(&scholar_teleop::ScholarTeleopNode::keyboardLoop, &scholar_teleop_data));  
    ros::spin();  
      
    mover.interrupt();  
    mover.join();  
    scholar_teleop_data.stopRobot();  
    
    tcgetattr(0, &scholar_teleop_data.old_data); 

    return(0);  
}  
