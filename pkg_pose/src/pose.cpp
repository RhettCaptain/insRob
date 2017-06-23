#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <geometry_msgs/PointStamped.h>		
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>	
#include "pkg_msgs/MsgServerCmd.h"
#include <cmath>		

ros::Publisher reviseOdometryPublisher;
ros::Publisher robotPosePublisher;

const double arc2deg = 180/M_PI;

float odomX;
float odomY;
float odomTh;
float odomVx;
float odomVy;
float odomVth;

float robotX;
float robotY;
float robotTh;

bool laserOn = false;

//odom-map转换监听，并发布odom的地图坐标
void transOdom(const tf::TransformListener& listener)
{
	geometry_msgs::PoseStamped odomPose,mapPose;
	odomPose.header.stamp = ros::Time();
	odomPose.header.frame_id = "odom";
	odomPose.pose.position.x = odomX;
	odomPose.pose.position.y = odomY;
	odomPose.pose.position.z = 0;
	odomPose.pose.orientation = tf::createQuaternionMsgFromYaw(odomTh);
	//转换
	try{
		listener.transformPose("map",odomPose,mapPose);
		//发布
		geometry_msgs::PoseWithCovarianceStamped robotPose;
		robotPose.header.stamp = ros::Time();
		robotPose.header.frame_id = "map";
		robotPose.pose.pose = mapPose.pose;
		robotPosePublisher.publish(robotPose);
		//更新robot pose
		robotX = mapPose.pose.position.x;
		robotY = mapPose.pose.position.y;
		robotTh = tf::getYaw(mapPose.pose.orientation);
	}
	catch(tf::TransformException& ex){
	}
	
	
}

//odometry数据更新
void handleOdom(const nav_msgs::Odometry::ConstPtr& msg)
{
	odomX = msg->pose.pose.position.x;
	odomY = msg->pose.pose.position.y;
	odomTh = tf::getYaw(msg->pose.pose.orientation);
	odomVx = msg->twist.twist.linear.x;
	odomVy = msg->twist.twist.linear.y;
	odomVth = msg->twist.twist.angular.z;

	if(!laserOn)
	{
		geometry_msgs::PoseWithCovarianceStamped robotPose;
		robotPose.header.stamp = ros::Time();
		robotPose.header.frame_id = "odom";
		robotPose.pose.pose.position.x = odomX;
		robotPose.pose.pose.position.y = odomY;
		robotPose.pose.pose.position.z = 0;
		robotPose.pose.pose.orientation = tf::createQuaternionMsgFromYaw(odomTh);
		robotPosePublisher.publish(robotPose);
	}


}

//amcl数据直接发布
void handleAmcl(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	robotPosePublisher.publish(*msg);

	//更新robot pose
	robotX = msg->pose.pose.position.x;
	robotY = msg->pose.pose.position.y;
	robotTh = tf::getYaw(msg->pose.pose.orientation);
}

//监听激光开关
void listenLaser(const pkg_msgs::MsgServerCmd::ConstPtr& msg)
{
	
	if(msg->cmd == "LASER_ON")
	{
		laserOn = true;
	}
	else if(msg->cmd == "LASER_OFF")
	{
		laserOn = false;
		//更新odom坐标
		geometry_msgs::PoseWithCovarianceStamped robotPose;
		robotPose.header.stamp = ros::Time();
		robotPose.header.frame_id = "odom";
		robotPose.pose.pose.position.x = robotX;
		robotPose.pose.pose.position.y = robotY;
		robotPose.pose.pose.position.z = 0;
		robotPose.pose.pose.orientation = tf::createQuaternionMsgFromYaw(robotTh);
		reviseOdometryPublisher.publish(robotPose);
	}
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"node_pose");
	ros::NodeHandle nodeHandle;
	ros::Subscriber odomSubscriber = nodeHandle.subscribe("odom",1000,handleOdom);		//订阅量测法数据
	ros::Subscriber amclSubscriber = nodeHandle.subscribe("amcl_pose",1000,handleAmcl);	//订阅激光定位数据
	ros::Subscriber serverCmdSubscriber = nodeHandle.subscribe("topic_server_cmd",1000,listenLaser);	//订阅服务器指令,监听激光开关
	robotPosePublisher = nodeHandle.advertise<geometry_msgs::PoseWithCovarianceStamped>("topic_robot_pose",1000);	//发布机器人位姿
	tf::TransformListener listener(ros::Duration(0.05));
	ros::Timer timer = nodeHandle.createTimer(ros::Duration(0.05),boost::bind(&transOdom, boost::ref(listener)));
	reviseOdometryPublisher = nodeHandle.advertise<geometry_msgs::PoseWithCovarianceStamped>("topic_revise_odometry",1000);	//发布量测修正信息
	ros::MultiThreadedSpinner spinner(6);	
	spinner.spin();
}
