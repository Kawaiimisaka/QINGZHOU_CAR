#ifndef AI_LINUX_SERIAL_H
#define AI_LINUX_SERIAL_H

#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <boost/asio.hpp>
#include <geometry_msgs/Twist.h>

extern void serialInit();
extern void writeSpeed(double RobotV, double YawRate);
extern bool readSpeed (double &vx,double &vth,double &delta_Distance,double &delta_th);



#endif
