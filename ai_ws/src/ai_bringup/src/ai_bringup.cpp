#include <ai_bringup/ai.h>

double RobotV_ = 0;
double RobotYawRate_ = 0;

// 速度控制消息的回调函数
void cmdCallback(const geometry_msgs::Twist& msg)
{
    RobotV_  = msg.linear.x;//m/s
    RobotYawRate_ = msg.angular.z;//rad/s
}

int main(int argc, char** argv)
{
	
    //初始化ROS节点
    ros::init(argc, argv, "serialPort");
    ros::NodeHandle nh;
    robot::robot myrobot;
    if(!myrobot.init())
    ROS_ERROR("myrobot initialized failed.");
    ROS_INFO("myrobot initialized successful.");
    
    ros::Subscriber sub = nh.subscribe("cmd_vel", 50, cmdCallback);
 

    //循环运行
    ros::Rate loop_rate(50);
 
        
        while(ros::ok())
        {
			ros::spinOnce();
			myrobot.deal(RobotV_, RobotYawRate_);
			loop_rate.sleep();
        
	    }
	


        
        
   

    return 0;
}

