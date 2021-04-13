#include <vector>
#include "ai_bringup/ai.h"
#include "ai_bringup/ai_linux_serial.h"
using namespace std;


namespace robot
{
    boost::array<double, 36> odom_pose_covariance = {
        {1e-9, 0, 0, 0, 0, 0, 
        0, 1e-3,1e-9, 0, 0, 0, 
        0, 0, 1e6, 0, 0, 0,
        0, 0, 0, 1e6, 0, 0, 
        0, 0, 0, 0, 1e6, 0, 
        0, 0, 0, 0, 0, 1e-9}};
    boost::array<double, 36> odom_twist_covariance = {
        {1e-9, 0, 0, 0, 0, 0, 
        0, 1e-3,1e-9, 0, 0, 0, 
        0, 0, 1e6, 0, 0, 0, 
        0, 0, 0, 1e6, 0, 0, 
        0, 0, 0, 0, 1e6, 0, 
        0, 0, 0, 0, 0, 1e-9}};
    robot::robot():x_(0.0), y_(0.0), th_(0.0),vx_(0.0), vy_(0.0), vth_(0.0),delta_Distance(0.0),delta_th(0.0){}//构造函数
    robot::~robot(){}                                                                                   //析构函数
    
    
    /********************************************************
    函数功能：串口参数初始化、时间变量初始化、实例化发布对象
    入口参数：无
    出口参数：bool
    ********************************************************/
     bool robot::init()
    {
        // 串口初始化连接
        serialInit();
               
        ros::Time::init();


        //定义发布消息的名称
        pub_ = nh.advertise<nav_msgs::Odometry>("odom", 50);		
        
        return true;
    }
   
    
    
    /********************************************************
    函数功能：根据机器人线速度和角度计算机器人里程计
    入口参数：无
    出口参数：无
    ********************************************************/
    void robot::calcOdom()
    {
		if (delta_th != 0)
		{
			th_ += delta_th;
		}
		
		if (delta_Distance != 0)
		{
			x_ += delta_Distance * cos(th_);
			y_ += delta_Distance * sin(th_);
		}
		

        //打印位姿调试信息，不用的时候可以关闭
        ROS_INFO("x_:%f\n",x_);
        ROS_INFO("y_:%f\n",y_);
        ROS_INFO("th_:%f\n",th_);
    }
    
    
    void robot::pubOdomAndTf()
    {
        current_time_ = ros::Time::now();
        // 发布TF
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time_;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id  = "base_footprint";

        geometry_msgs::Quaternion odom_quat;
        odom_quat = tf::createQuaternionMsgFromYaw(th_);
        odom_trans.transform.translation.x = x_;
        odom_trans.transform.translation.y = y_;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;
        
        odom_broadcaster_.sendTransform(odom_trans);

        // 发布里程计消息
        nav_msgs::Odometry msgl;
        msgl.header.stamp = current_time_;
        msgl.header.frame_id = "odom";

        msgl.pose.pose.position.x = x_;
        msgl.pose.pose.position.y = y_;
        msgl.pose.pose.position.z = 0.0;
        msgl.pose.pose.orientation = odom_quat;
        msgl.pose.covariance = odom_pose_covariance;

        msgl.child_frame_id = "base_footprint";
        msgl.twist.twist.linear.x = vx_;
        msgl.twist.twist.linear.y = vy_;
        msgl.twist.twist.angular.z = vth_;
        msgl.twist.covariance = odom_twist_covariance;
    
        pub_.publish(msgl);

    }
    
    
        /********************************************************
    函数功能：自定义deal，实现整合，并且发布TF变换和Odom
    入口参数：机器人线速度和角速度，调用上面三个函数
    出口参数：bool
    ********************************************************/
     bool robot::deal(double RobotV, double RobotYawRate)
    {
        // 向STM32发送对机器人的预期控制速度，以及预留信号控制位
        writeSpeed(RobotV, RobotYawRate);
        // 从STM32读取机器人实际线速度，角速度和角度，以及预留信号控制位
        readSpeed(vx_, vth_,delta_Distance,delta_th);
        // 里程计计算
        calcOdom();
        // 发布TF变换和Odom
        pubOdomAndTf();
    }
    

}

