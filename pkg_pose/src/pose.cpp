#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <geometry_msgs/PointStamped.h>		
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>			

ros::Publisher reviseOdometryPublisher;
ros::Publisher robotPosePublisher;

float x;
float y;
float th;
float vx;
float vy;
float vth;

//odom-map转换监听，并发布odom的地图坐标
void transOdom(const tf::TransformListener& listener)
{
	geometry_msgs::PoseStamped odomPose,mapPose;
	odomPose.header.stamp = ros::Time();
	odomPose.header.frame_id = "odom";
	odomPose.pose.position.x = x;
	odomPose.pose.position.y = y;
	odomPose.pose.position.z = 0;
	odomPose.pose.orientation = tf::createQuaternionMsgFromYaw(th);
	//转换
	listener.transformPose("map",odomPose,mapPose);
	
	
}

//odometry数据更新
void handleOdom(const nav_msgs::Odometry::ConstPtr& msg)
{
	x = msg->pose.pose.position.x;
	y = msg->pose.pose.position.y;
	th = tf::getYaw(msg->pose.pose.orientation);
	vx = msg->twist.twist.linear.x;
	vy = msg->twist.twist.linear.y;
	vth = msg->twist.twist.angular.z;
}

//amcl数据直接发布
void handleAmcl(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	robotPosePublisher.publish(*msg);
	/*
	float x = msg->pose.pose.position.x;
	float y = msg->pose.pose.position.y;
	float th = tf::getYaw(msg->pose.pose.orientation);
	//更新
	//不要更新
	
	geometry_msgs::PoseWithCovarianceStamped reviseMsg;
	reviseMsg.pose.pose.position.x = x;
	reviseMsg.pose.pose.position.y = y;
	reviseMsg.pose.pose.orientation.z = th;
	reviseOdometryPublisher.publish(reviseMsg);*/
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"node_pose");
	ros::NodeHandle nodeHandle;
	ros::Subscriber odomSubscriber = nodeHandle.subscribe("odom",1000,handleOdom);		//订阅量测法数据
	ros::Subscriber amclSubscriber = nodeHandle.subscribe("amcl_pose",1000,handleAmcl);	//订阅激光定位数据
	robotPosePublisher = nodeHandle.advertise<geometry_msgs::PoseWithCovarianceStamped>("topic_robot_pose",1000);	//发布量测修正信息
//	reviseOdometryPublisher = nodeHandle.advertise<geometry_msgs::PoseWithCovarianceStamped>("topic_revise_odometry",1000);	//发布量测修正信息	
	ros::spin();
}
