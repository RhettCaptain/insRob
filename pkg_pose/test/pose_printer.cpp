#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <cmath>
#include <tf/transform_broadcaster.h>

const double arc2deg = 180/M_PI;
//odometry数据处理函数
void handleOdom(const nav_msgs::Odometry::ConstPtr& msg)
{
	float x = msg->pose.pose.position.x;
	float y = msg->pose.pose.position.y;
	float th = tf::getYaw(msg->pose.pose.orientation);
	float vx = msg->twist.twist.linear.x;
	float vy = msg->twist.twist.linear.y;
	float vth = msg->twist.twist.angular.z;
	printf("get odom data:x%f,y%f,th%f\n",x,y,th*arc2deg);
}

//amcl数据处理函数
void handleAmcl(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	float x = msg->pose.pose.position.x;
	float y = msg->pose.pose.position.y;
	float th = tf::getYaw(msg->pose.pose.orientation);
	printf("get amcl data:x%f,y%f,th%f\n",x,y,th*arc2deg);
}

//正常输出robot_pose
void handlePose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	float x = msg->pose.pose.position.x;
	float y = msg->pose.pose.position.y;
	float th = tf::getYaw(msg->pose.pose.orientation);
	printf("get robot pose data:x%f,y%f,th%f\n",x,y,th*arc2deg);
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"node_pose_printer");
	ros::NodeHandle nodeHandle;
	ros::Subscriber amclSubscriber;
	ros::Subscriber odomSubscriber;
	ros::Subscriber poseSubscriber;
	if(argc>1)
	{
		if(strcmp(argv[1],"amcl") == 0)
		{
			amclSubscriber = nodeHandle.subscribe("amcl_pose",1000,handleAmcl);
		}
		else if(strcmp(argv[1],"odom") == 0)
		{
			odomSubscriber = nodeHandle.subscribe("odom",1000,handleOdom);
		}
		else if(strcmp(argv[1],"pose") == 0)
		{
			poseSubscriber = nodeHandle.subscribe("topic_robot_pose",1000,handlePose);
		}
		else if(strcmp(argv[1],"all") == 0)
		{
			amclSubscriber = nodeHandle.subscribe("amcl_pose",1000,handleAmcl);
			odomSubscriber = nodeHandle.subscribe("odom",1000,handleOdom);
			poseSubscriber = nodeHandle.subscribe("topic_robot_pose",1000,handlePose);
		}
		else
		{
			ROS_INFO("please input correct parameter!");
			exit(0);
		}
	}
	else
	{
		odomSubscriber = nodeHandle.subscribe("topic_robot_pose",1000,handlePose);
	}
	
	
	ros::spin();
}
