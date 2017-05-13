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

// position
double x; 
double y;
double th;

// velocity
double vx;
double vy;
double vth;

ros::Time curTime;
ros::Time lastTime;
	
void updateData(const pkg_msgs::MsgOdometrySensor::ConstPtr& msg)
{
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
std::cout<<"vl: " << vl << "vr: " <<vr<<std::endl;
vth = (vr-vl)/0.36;
	//	x += vx * dt;
	//	y += vy * dt;
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

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "state_publisher");
	ros::NodeHandle n;
	ros::NodeHandle n2;
ros::Subscriber odometrySensorSubscriber = n2.subscribe("topic_odometry_sensor",1000,updateData);	//订阅量测传感器信息
ros::Subscriber reviseOdometrySubscriber = n.subscribe("topic_revise_odometry",1000,reviseOdometry);	//订阅量测修正信息
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);

	// initial position
	x = 0.0; 
	y = 0.0;
	th = 0;

	// velocity
	vx = 0.0;
	vy = 0.0;
	vth = 0.0;


	curTime = ros::Time::now();
	lastTime = ros::Time::now();

	tf::TransformBroadcaster broadcaster;
	ros::Rate loop_rate(20);

	const double degree = M_PI/180;

	// message declarations
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";

	while (ros::ok()) {
		curTime = ros::Time::now(); 
        	ros::spinOnce();
		double dt = (curTime - lastTime).toSec();
		double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
		double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
		double delta_th = vth * dt;

		x += delta_x;
		y += delta_y;
		th += delta_th;
std::cout << "x: " << x << "y: " << y <<"th " << th/degree << std::endl;
std::cout << "vx: " << vx << "vy: " << vy << std::endl;
		geometry_msgs::Quaternion odom_quat;	
		odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,th);

		// update transform
		odom_trans.header.stamp = curTime; 
		odom_trans.transform.translation.x = x;  
		odom_trans.transform.translation.y = y;  
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(th);

		//filling the odometry
		nav_msgs::Odometry odom;
		odom.header.stamp = curTime;
		odom.header.frame_id = "odom";
		odom.child_frame_id = "base_link";

		// position
		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;

		//velocity
		odom.twist.twist.linear.x = vx;
		odom.twist.twist.linear.y = vy;
		odom.twist.twist.linear.z = 0.0;
		odom.twist.twist.angular.x = 0.0;
		odom.twist.twist.angular.y = 0.0;
		odom.twist.twist.angular.z = vth;

		lastTime = curTime;

		// publishing the odometry and the new tf
		broadcaster.sendTransform(odom_trans);
	//	odom_pub.publish(odom);

		loop_rate.sleep();
	}
	return 0;

}

