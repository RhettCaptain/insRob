#ifndef ODOM_H
#define ODOM_H


#include <string>
#include <ros/ros.h>
#include <pkg_msgs/MsgOdometrySensor.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <math.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <iostream>


class Odom
{
private:
	double x,y,th;
	double vx,vy,vth;
	
	ros::Publisher odometryPublisher;
	const double degree = M_PI/180;
	ros::Time curTime;
	ros::Time lastTime;
	geometry_msgs::TransformStamped odom_tf;
	nav_msgs::Odometry odometry;

	void updateData(const pkg_msgs::MsgOdometrySensor::ConstPtr& msg); 
	void reviseOdometry(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
public:
	Odom();
	~Odom();
	void publish();	
};



#endif
