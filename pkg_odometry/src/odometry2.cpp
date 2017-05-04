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


double x,y,th;			//位姿
double vx,vy,vth;		//速度

const double degree = M_PI/180;
ros::Time curTime;
ros::Time lastTime;

void updateData(const pkg_msgs::MsgOdometrySensor::ConstPtr& msg)
{
//	std::cout << "updateData\n";
	curTime = ros::Time::now(); 
	//读取、判断、更新最新数据
	if(msg->type == "ODOMETER")
	{	
		double vlf = msg->vlf;
		double vlb = msg->vlb;
		double vrf = msg->vrf;
		double vrb = msg->vrb;
		double vl = (vlf+vlb)/2;
		double vr = (vrf+vrb)/2;
		double v = (vl+vr)/2;
		double dt = (curTime - lastTime).toSec();
		vx = cos(th) * v;
		vy = sin(th) * v;
		//vth remain
		x += vx * dt;
		y += vy * dt;
		//th remain 
	}
	else if(msg->type == "COMPASS")
	{
		double dt = (curTime - lastTime).toSec();
		//vx remain
		//vy remain
		vth = (msg->th - th)/dt;
		x += vx * dt;
		y += vy * dt;
		th = msg->th; 
	}
	else if(msg->type == "IMU")
	{
		//TO DO
	}

	lastTime = curTime;
}

void reviseOdometry(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	curTime = ros::Time::now(); 
	x = msg->pose.pose.position.x;
	y = msg->pose.pose.position.y;
	th = tf::getYaw(msg->pose.pose.orientation);
	lastTime = curTime;
	
}
int main(int argc, char** argv) {
	ros::init(argc,argv,"node_odometry");
	ros::NodeHandle nodeHandle;
	curTime = ros::Time::now(); 
	lastTime = ros::Time::now(); 
	ros::Publisher odometryPublisher;	//量测信息发布器
	tf::TransformBroadcaster tfBroadcaster;	//坐标转换广播器
	geometry_msgs::TransformStamped odom_tf;
    nav_msgs::Odometry odometry;
	ros::Subscriber odometrySensorSubscriber = nodeHandle.subscribe("topic_odometry_sensor",1000,updateData);	//订阅量测传感器信息
	ros::Subscriber reviseOdometrySubscriber = nodeHandle.subscribe("topic_revise_odometry",1000,reviseOdometry);	//订阅量测修正信息
	odometryPublisher = nodeHandle.advertise<nav_msgs::Odometry>("odom",10);		
	
	//读取初始位姿
	std::string userName = getlogin();
	std::string fileName = "/home/" + userName + "/.nav/initPose.cfg";
	int initPoseFd = open(fileName.c_str(),O_RDONLY);
	if(initPoseFd<0)	//没有文件则取0
	{
		x = 0.0;
		y = 0.0;
		th = 0.0;
	}
	else
	{	
		th = atof(strtok(NULL,";"));
	}
	
	while(ros::ok())
	{
	    ros::spinOnce();
		//发布最新量测法信息
	    //填入头信息
	    odometry.header.stamp = curTime;
	    odometry.header.frame_id = "odom";
	    odometry.child_frame_id = "base_link";
	    //更新位姿
	    odometry.pose.pose.position.x = 0;//x;
	    odometry.pose.pose.position.y = 0;//y;
	    odometry.pose.pose.position.z = 0.0;
	    geometry_msgs::Quaternion odometry_quat;	
	    odometry_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);//th);
	    odometry.pose.pose.orientation = odometry_quat;
	    //更新速度
	    odometry.twist.twist.linear.x = 0;//vx;
	    odometry.twist.twist.linear.y = 0;//vy;
	    odometry.twist.twist.linear.z = 0.0;
	    odometry.twist.twist.angular.x = 0.0;
	    odometry.twist.twist.angular.y = 0.0;
	    odometry.twist.twist.angular.z = 0;//vth;
	    //发布
	    odometryPublisher.publish(odometry);
        
	    //发布最新"odom->base_link"坐标转换
	    odom_tf.header.stamp = curTime; 
	    odom_tf.header.frame_id = "odom";
	    odom_tf.child_frame_id = "base_link";
	    odom_tf.transform.translation.x = 0;//x; 
	    odom_tf.transform.translation.y = 0;//y; 
	    odom_tf.transform.translation.z = 0.0;
	    odom_tf.transform.rotation = tf::createQuaternionMsgFromYaw(0);//th);	
	    tfBroadcaster.sendTransform(odom_tf);
	}
	
	return 0;
	
}

