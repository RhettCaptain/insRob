#include "ros/ros.h"
#include "pkg_srvs/SrvMode.h"
#include "pkg_srvs/SrvGetLine.h"
#include "pkg_srvs/SrvGetLineTheta.h"
#include <stdlib.h>
#include <unistd.h>
#include <string>
#include <pthread.h>
#include "pkg_msgs/MsgServerCmd.h"
#include "pkg_srvs/SrvGetDistance.h"
#include "pkg_srvs/SrvPointLine.h"
#include "pkg_srvs/SrvGetYawBias.h"
#include <math.h>
#include <tf/transform_broadcaster.h>




using namespace std;

pkg_msgs::MsgServerCmd cmdMsg;
ros::Publisher cmdPublisher;
bool changeMode(pkg_srvs::SrvMode::Request &req,pkg_srvs::SrvMode::Response &res)
{
	string cmd = req.cmd;
	res.state = false;
	if(cmd == "MAP_ON")
	{
		cmdMsg.cmd = "MAP_ON";
		cmdPublisher.publish(cmdMsg);
		res.state = true;
	}
	else if(cmd == "MAP_OFF")
	{
		int child;
		child = system("rosrun map_server map_saver -f ~/.nav/slam_map");
		if(child<0)
		{
			return false;
		}
		child = system("sh ~/.nav/kill_process.sh slam_gmapping");
		if(child<0)
		{
			return false;
		}
		child = system("sh ~/.nav/kill_process.sh laser_node");
		if(child<0)
		{
			return false;
		}
		child = system("sh ~/.nav/kill_process.sh node_odometry");
		if(child<0)
		{
			return false;
		}
		child = system("sh ~/.nav/kill_process.sh node_sensors_tf");
		if(child<0)
		{
			return false;
		}
		res.state = true;
	}
	else if(cmd == "ODOM_ON")
	{
		cmdMsg.cmd = "ODOM_ON";
		cmdPublisher.publish(cmdMsg);
		res.state = true;
	}
	else if(cmd == "ODOM_OFF")
	{
		int child;
		child = system("sh ~/.nav/kill_process.sh node_pose");
		if(child<0)
		{
			return false;
		}
		child = system("sh ~/.nav/kill_process.sh node_odometry");
		if(child<0)
		{
			return false;
		}
		child = system("sh ~/.nav/kill_process.sh node_sensors_tf");
		if(child<0)
		{
			return false;
		}
		res.state = true;
	}
	else if(cmd == "LASER_ON")
	{
		cmdMsg.cmd = "LASER_ON";
		cmdPublisher.publish(cmdMsg);
		res.state = true;
	}
	else if(cmd == "LASER_OFF")
	{
		int child;
		cmdMsg.cmd = "LASER_OFF";
		cmdPublisher.publish(cmdMsg);
		child = system("sh ~/.nav/kill_process.sh amcl");
		if(child<0)
		{
			return false;
		}
		child = system("sh ~/.nav/kill_process.sh map_server");
		if(child<0)
		{
			return false;
		}
		child = system("sh ~/.nav/kill_process.sh laser_node");
		if(child<0)
		{
			return false;
		}
		res.state = true;
	}
	return true;

}

bool getLine(pkg_srvs::SrvGetLine::Request &req,pkg_srvs::SrvGetLine::Response &res)
{
	double x1, y1, x2, y2;
	x1=req.poseA.pose.position.x;
	y1=req.poseA.pose.position.y;	
	x2=req.poseB.pose.position.x;
	y2=req.poseB.pose.position.y;
	double x, y,x0,y0;
	x = x2 - x1;
	y = y2 - y1;
	x0 = 1; y0 = 0;
	double ab, aa, bb, cosr, ltheta, linetheta;
	ab = x * x0 + y * y0;
	aa = sqrt(x * x + y * y);
	bb = sqrt(x0 * x0 + y0  * y0);
	cosr = ab / aa / bb;   //余弦
	ltheta = acos(cosr);
	if (y<0)
		linetheta = - ltheta;
	else
		linetheta = ltheta;
		
	res.line[0]=y2-y1;
        res.line[1]=x1-x2;
	res.line[2]=x2*y1-x1*y2;
	res.line[3]=linetheta;
        return true;
}

bool getLineTheta(pkg_srvs::SrvGetLineTheta::Request &req, pkg_srvs::SrvGetLineTheta::Response &res)
{
	double x1,y1,x2,y2;
	x1=req.poseA.pose.position.x;
	y1=req.poseA.pose.position.y;	
	x2=req.poseB.pose.position.x;
	y2=req.poseB.pose.position.y;
        double x, y,x0,y0;
	x = x2 - x1;
	y = y2 - y1;
	x0 = 1; y0 = 0;
	double ab, aa, bb, cosr, ltheta, linetheta;
	ab = x * x0 + y * y0;
	aa = sqrt(x * x + y * y);
	bb = sqrt(x0 * x0 + y0  * y0);
	cosr = ab / aa / bb;
	ltheta = acos(cosr);
	if (y>0)
		linetheta = - ltheta;
	else
		linetheta = ltheta;
		
	res.theta=linetheta;
	return true;
	
}

bool pointLine(pkg_srvs::SrvPointLine::Request &req, pkg_srvs::SrvPointLine::Response &res)
{
	double x,y,a,b,c;
	x=req.pose.pose.position.x;
	y=req.pose.pose.position.y;
	a=req.line[0];
	b=req.line[1];
	c=req.line[2];
	res.distance=(a*x + b*y + c)/ sqrt(a*a + b*b);
  	return true;
}


bool getYawBias(pkg_srvs::SrvGetYawBias::Request  &req, pkg_srvs::SrvGetYawBias::Response &res)
{
	double x1, y1, x2, y2,bias;
	x1=req.poseA.pose.position.x;
	y1=req.poseA.pose.position.y;	
	x2=req.poseB.pose.position.x;
	y2=req.poseB.pose.position.y;
        bias=tf::getYaw(req.poseC.pose.orientation);
        if(bias<0)
  	{
  	    bias=2 * M_PI+bias;
  	}
        double x, y,x0,y0;   
	x = x2 - x1;
	y = y2 - y1;
	x0 = 1; y0 = 0;
	double ab, aa, bb, cosr, ltheta, linetheta, yaws;
	ab = x * x0 + y * y0;
	aa = sqrt(x * x + y * y);
	bb = sqrt(x0 * x0 + y0  * y0);
	cosr = ab / aa / bb;
	ltheta = acos(cosr);
	if (y<0)
		linetheta = 2 * M_PI - ltheta;
	else
		linetheta = ltheta;
	double yaw;
	yaw= bias - linetheta; 
	double th;
	if (yaw > M_PI)
		th = yaw -2* M_PI;
	else if (yaw < -M_PI)
		th = 2 * M_PI + yaw;
	else
		th = yaw;
	res.theta= -th;
	return true;
}

bool getDistance(pkg_srvs::SrvGetDistance::Request  &req, pkg_srvs::SrvGetDistance::Response &res)
{
	double x1,y1,x2,y2;
	x1=req.poseA.pose.position.x;
	y1=req.poseA.pose.position.y;	
	x2=req.poseB.pose.position.x;
	y2=req.poseB.pose.position.y;
	res.distance=sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
 
  	return true;
}


int main(int argc, char** argv)
{
	ros::init(argc,argv,"node_server");
	ros::NodeHandle nodeHandle;
	ros::ServiceServer modeService = nodeHandle.advertiseService("srv_mode",changeMode);
	ros::ServiceServer getLineService = nodeHandle.advertiseService("srv_get_line",getLine);
	ros::ServiceServer getLineThetaService = nodeHandle.advertiseService("srv_get_line_theta",getLineTheta);
	ros::ServiceServer pointLineService = nodeHandle.advertiseService("srv_point_line",pointLine);
	ros::ServiceServer getYawBiasService = nodeHandle.advertiseService("srv_get_yaw_bias",getYawBias);
	ros::ServiceServer getDistanceService = nodeHandle.advertiseService("srv_get_distance",getDistance);
	cmdPublisher = nodeHandle.advertise<pkg_msgs::MsgServerCmd>("topic_server_cmd",1000);
	ros::spin();
	return 0;
}
