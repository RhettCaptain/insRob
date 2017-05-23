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

//正常输出robot_pose
void handleAmcl(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	float x = msg->pose.pose.position.x;
	float y = msg->pose.pose.position.y;
	float th = msg->pose.pose.orientation.z;
	printf("get robot pose data:x%f,y%f,th%f\n",x,y,th);
}

//amcl数据处理函数
void handlePose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	float x = msg->pose.pose.position.x;
	float y = msg->pose.pose.position.y;
	float th = msg->pose.pose.orientation.z;
	printf("get amcl data:x%f,y%f,th%f\n",x,y,th);
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
		if(argv[1] == "amcl")
		{
			amclSubscriber = nodeHandle.subscribe("amcl_pose",1000,handleAmcl);
		}
		else if(argv[1] == "odom")
		{
			odomSubscriber = nodeHandle.subscribe("odom",1000,handleOdom);
		}
		else if(argv[1] == "pose")
		{
			poseSubscriber = nodeHandle.subscribe("topic_robot_pose",1000,handlePose);
		}
		else if(argv[1] == "all")
		{
			amclSubscriber = nodeHandle.subscribe("amcl_pose",1000,handleAmcl);
			odomSubscriber = nodeHandle.subscribe("odom",1000,handleOdom);
			poseSubscriber = nodeHandle.subscribe("topic_robot_pose",1000,handlePose);
		}
		else
		{
			ROS_INFO("please input correct parameter!");
		}
	}
	else
	{
		odomSubscriber = nodeHandle.subscribe("topic_robot_pose",1000,handlePose);
	}
	
	
	ros::spin();
}
