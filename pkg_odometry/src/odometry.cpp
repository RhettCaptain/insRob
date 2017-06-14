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


const double wheelDis = 0.36;
void updateData(const pkg_msgs::MsgOdometrySensor::ConstPtr& msg)
{
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
		vx = cos(th) * v;
		vy = sin(th) * v;
std::cout<<"vl: " << vl << "vr: " <<vr<<std::endl;
		vth = (vr-vl)/wheelDis;
	//	x += vx * dt;
	//	y += vy * dt;
		//th remain 
	}
	else if(msg->type == "COMPASS")
	{
		//TO DO
	}
	else if(msg->type == "IMU")
	{
		//TO DO
	}
}


int main(int argc, char** argv) 
{
    	ros::init(argc, argv, "state_publisher");
	ros::NodeHandle n;
	ros::NodeHandle n2;
	ros::Subscriber odometrySensorSubscriber = n2.subscribe("topic_odometry_sensor",1000,updateData);	//订阅量测传感器信息
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);

	// initial position
	x = 0.0; 
	y = 0.0;
	th = 0;

	// velocity
	vx = 0.0;
	vy = 0.0;
	vth = 0.0;

	ros::Time curTime = ros::Time::now();
	ros::Time lastTime = ros::Time::now();

	tf::TransformBroadcaster broadcaster;
	ros::Rate loop_rate(100);

	const double arc2deg = 180/M_PI;

	// message declarations
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";

	while (ros::ok()) {
		//周期性航迹推算
		curTime = ros::Time::now(); 	//周期开始，更新当前时间
        	ros::spinOnce();		//检查跟新外部数据
		double dt = (curTime - lastTime).toSec();
		double delta_x = vx * dt;
		double delta_y = vy * dt;
		double delta_th = vth * dt;
		x += delta_x;
		y += delta_y;
		th += delta_th;
		//角度范围-pi - pi，弧度制
		if(th < -M_PI)
		{
			th += 2*M_PI;
		}
		else if(th > M_PI)
		{
			th -= 2*M_PI;
		}
		
		lastTime = curTime;		//航迹推算处理结束，当前时间作为上一周期时间。
//虽然一次循环没有结束，但位姿的新周期已经开始了。
std::cout << "x: " << x << "y: " << y <<"th " << th * arc2deg << std::endl;
std::cout << "vx: " << vx << "vy: " << vy << std::endl;

		//更新base_link->odom坐标变换
		odom_trans.header.stamp = curTime; 
		odom_trans.transform.translation.x = x;  
		odom_trans.transform.translation.y = y;  
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(th);

		//更新odom消息
		nav_msgs::Odometry odom;
		odom.header.stamp = curTime;
		odom.header.frame_id = "odom";
		odom.child_frame_id = "base_link";
		//位姿
		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(th);
		//速度
		odom.twist.twist.linear.x = vx;
		odom.twist.twist.linear.y = vy;
		odom.twist.twist.linear.z = 0.0;
		odom.twist.twist.angular.x = 0.0;
		odom.twist.twist.angular.y = 0.0;
		odom.twist.twist.angular.z = vth;

		//发布odom消息和baselink->odom坐标变换
		broadcaster.sendTransform(odom_trans);
		odom_pub.publish(odom);

		loop_rate.sleep();
	}
	return 0;

}

