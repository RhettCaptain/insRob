#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <geometry_msgs/PointStamped.h>		
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>	
#include <cmath>		

ros::Publisher reviseOdometryPublisher;
ros::Publisher robotPosePublisher;

const double arc2deg = 180/M_PI;


//odometry数据更新,并发布
void handleOdom(const nav_msgs::Odometry::ConstPtr& msg)
{
	geometry_msgs::PoseWithCovarianceStamped robotPose;
	robotPose.header.stamp = ros::Time();
	robotPose.header.frame_id = "map";
	robotPose.pose.pose.position.x = msg->pose.pose.position.x;
	robotPose.pose.pose.position.y = msg->pose.pose.position.y;
	robotPose.pose.pose.position.z = 0;
	robotPose.pose.pose.orientation = msg->pose.pose.orientation;
	robotPosePublisher.publish(robotPose);
}

//amcl数据直接发布,并修正odom
void handleAmcl(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	robotPosePublisher.publish(*msg);

	float x = msg->pose.pose.position.x;
	float y = msg->pose.pose.position.y;
	float th = tf::getYaw(msg->pose.pose.orientation);
	geometry_msgs::PoseWithCovarianceStamped reviseMsg;
	reviseMsg.pose.pose.position.x = x;
	reviseMsg.pose.pose.position.y = y;
	reviseMsg.pose.pose.position.z = 0;
	reviseMsg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(th);
	reviseOdometryPublisher.publish(reviseMsg);
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"node_pose");
	ros::NodeHandle nodeHandle;
	ros::Subscriber odomSubscriber = nodeHandle.subscribe("odom",1000,handleOdom);		//订阅量测法数据
	ros::Subscriber amclSubscriber = nodeHandle.subscribe("amcl_pose",1000,handleAmcl);	//订阅激光定位数据
	robotPosePublisher = nodeHandle.advertise<geometry_msgs::PoseWithCovarianceStamped>("topic_robot_pose",1000);	//发布机器人位姿
	//tf::TransformListener listener(ros::Duration(0.05));
	//ros::Timer timer = nodeHandle.createTimer(ros::Duration(0.05),boost::bind(&transOdom, boost::ref(listener)));
	reviseOdometryPublisher = nodeHandle.advertise<geometry_msgs::PoseWithCovarianceStamped>("topic_revise_odometry",1000);	//发布量测修正信息	
	ros::spin();
}
