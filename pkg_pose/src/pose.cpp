#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf/transform_broadcaster.h>

ros::Publisher reviseOdometryPublisher;

//odometry数据处理函数
void handleOdom(const nav_msgs::Odometry::ConstPtr& msg)
{
	float x = msg->pose.pose.position.x;
	float y = msg->pose.pose.position.y;
	float th = tf::getYaw(msg->pose.pose.orientation);
	float vx = msg->twist.twist.linear.x;
	float vy = msg->twist.twist.linear.y;
	float vth = msg->twist.twist.angular.z;
}

//amcl数据处理函数
void handleAmcl(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	float x = msg->pose.pose.position.x;
	float y = msg->pose.pose.position.y;
	float th = tf::getYaw(msg->pose.pose.orientation);
	//更新
	geometry_msgs::PoseWithCovarianceStamped reviseMsg;
	reviseMsg.pose.pose.position.x = x;
	reviseMsg.pose.pose.position.y = y;
	reviseMsg.pose.pose.orientation.z = th;
	reviseOdometryPublisher.publish(reviseMsg);
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"node_pose");
	ros::NodeHandle nodeHandle;
	ros::Subscriber odomSubscriber = nodeHandle.subscribe("odom",1000,handleOdom);		//订阅量测法数据
	ros::Subscriber amclSubscriber = nodeHandle.subscribe("amcl_pose",1000,handleAmcl);	//订阅激光定位数据
	reviseOdometryPublisher = nodeHandle.advertise<geometry_msgs::PoseWithCovarianceStamped>("topic_revise_odometry",1000);	//发布量测修正信息	
	ros::spin();
}
