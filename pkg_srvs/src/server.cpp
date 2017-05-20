#include "ros/ros.h"
#include "pkg_srvs/SrvMode.h"
#include "pkg_srvs/SrvGetLine.h"
#include <stdlib.h>
#include <unistd.h>
#include <string>
#include <pthread.h>
#include "pkg_msgs/MsgServerCmd.h"


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
		child = system("sh ~/.nav/kill_process.sh LMS1xx_node");
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
		child = system("sh ~/.nav/kill_process.sh LMS1xx_node");
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
	res.line[0]=y2-y1;
  	res.line[1]=x1-x2;
	res.line[2]=x2*y1-x1*y2;
  	return true;
}



int main(int argc, char** argv)
{
	ros::init(argc,argv,"node_server");
	ros::NodeHandle nodeHandle;
	ros::ServiceServer modeService = nodeHandle.advertiseService("srv_mode",changeMode);
	ros::ServiceServer getLineService = nodeHandle.advertiseService("srv_get_line",getLine);
	cmdPublisher = nodeHandle.advertise<pkg_msgs::MsgServerCmd>("topic_server_cmd",1000);
	ros::spin();
	return 0;
}
