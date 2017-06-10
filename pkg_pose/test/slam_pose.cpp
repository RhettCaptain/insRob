#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <geometry_msgs/PointStamped.h>		
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>			

ros::Publisher reviseOdometryPublisher;
ros::Publisher robotPosePublisher;


//slam数据直接发布
void handleSlam(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	geometry_msgs::PoseWithCovarianceStamped robotPose;
	robotPose.pose.pose.position.x = msg->pose.position.x;
	robotPose.pose.pose.position.y = msg->pose.position.y;
	robotPose.pose.pose.orientation.z = msg->pose.orientation.z;
	robotPosePublisher.publish(robotPose);
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
	
	ros::Subscriber slamSubscriber = nodeHandle.subscribe("slam_out_pose",1000,handleSlam);	//订阅激光定位数据
	robotPosePublisher = nodeHandle.advertise<geometry_msgs::PoseWithCovarianceStamped>("topic_robot_pose",1000);	//发布量测修正信息
//	reviseOdometryPublisher = nodeHandle.advertise<geometry_msgs::PoseWithCovarianceStamped>("topic_revise_odometry",1000);	//发布量测修正信息	
	ros::spin();
}
