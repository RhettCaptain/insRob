#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

//odometry数据处理函数
void handleOdom(const nav_msgs::Odometry::ConstPtr& msg)
{
	float x = msg->pose.pose.position.x;
	float y = msg->pose.pose.position.y;
	float th = msg->pose.pose.orientation.z;
	float vx = msg->twist.twist.linear.x;
	float vy = msg->twist.twist.linear.y;
	float vth = msg->twist.twist.angular.z;
	printf("get odom data:x%f,y%f,th%f\n",x,y,th);
}

//amcl数据处理函数
void handleAmcl(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	float x = msg->pose.pose.position.x;
	float y = msg->pose.pose.position.y;
	float th = msg->pose.pose.orientation.z;
	printf("get amcl data:x%f,y%f,th%f\n",x,y,th);
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"node_pose_printer");
	ros::NodeHandle nodeHandle1,nodeHandle2;
	ros::Subscriber odomSubscriber = nodeHandle1.subscribe("odom",1000,handleOdom);
	ros::Subscriber amclSubscriber = nodeHandle2.subscribe("amcl_pose",1000,handleAmcl);
	ros::spin();
}
