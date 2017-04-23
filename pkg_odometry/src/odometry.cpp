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

double x,y,th;			//位姿
double vx,vy,vth;		//速度

ros::Publisher odometryPublisher;	//量测信息发布器
tf::TransformBroadcaster tfBroadcaster;	//坐标转换广播器
const double degree = M_PI/180;
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

	//发布最新量测法信息
	//填入头信息
	nav_msgs::Odometry odometry;
	odometry.header.stamp = curTime;
	odometry.header.frame_id = "odom";
	odometry.child_frame_id = "base_link";
	//更新位姿
	odometry.pose.pose.position.x = x;
	odometry.pose.pose.position.y = y;
	odometry.pose.pose.position.z = 0.0;
	geometry_msgs::Quaternion odometry_quat;	
	odometry_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,th);
	odometry.pose.pose.orientation = odometry_quat;
	//更新速度
	odometry.twist.twist.linear.x = vx;
	odometry.twist.twist.linear.y = vy;
	odometry.twist.twist.linear.z = 0.0;
	odometry.twist.twist.angular.x = 0.0;
	odometry.twist.twist.angular.y = 0.0;
	odometry.twist.twist.angular.z = vth;
	//发布
	odometryPublisher.publish(odometry);

	//发布最新"odom->base_link"坐标转换
	geometry_msgs::TransformStamped odom_tf;
	odom_tf.header.frame_id = "odom";
	odom_tf.child_frame_id = "base_link";
	odom_tf.header.stamp = curTime; 
	odom_tf.transform.translation.x = x; 
	odom_tf.transform.translation.y = y; 
	odom_tf.transform.translation.z = 0.0;
	odom_tf.transform.rotation = tf::createQuaternionMsgFromYaw(th);
	tfBroadcaster.sendTransform(odom_tf);

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
	
	ros::spin();	
	
	return 0;
	
/*   ref
	ros::init(argc, argv, "state_publisher");
	ros::NodeHandle n;
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);

	// initial position
	double x = 0.0; 
	double y = 0.0;
	double th = 0;

	// velocity
	double vx = 0.4;
	double vy = 0.0;
	double vth = 0.4;

	ros::Time current_time;
	ros::Time last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	tf::TransformBroadcaster broadcaster;
	ros::Rate loop_rate(20);

	const double degree = M_PI/180;

	// message declarations
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";

	while (ros::ok()) {
		current_time = ros::Time::now(); 

		double dt = (current_time - last_time).toSec();
		double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
		double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
		double delta_th = vth * dt;

		x += delta_x;
		y += delta_y;
		th += delta_th;

		geometry_msgs::Quaternion odom_quat;	
		odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,th);

		// update transform
		odom_trans.header.stamp = current_time; 
		odom_trans.transform.translation.x = x  
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(th);

		//filling the odometry
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
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

		last_time = current_time;

		// publishing the odometry and the new tf
		broadcaster.sendTransform(odom_trans);
		odom_pub.publish(odom);

		loop_rate.sleep();
	}
	return 0;
*/
}

