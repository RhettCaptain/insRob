#include "ros/ros.h"
#include "pkg_msgs/MsgServerCmd.h"


void openLaser(const pkg_msgs::MsgServerCmd::ConstPtr& msg)
{
	
	if(msg->cmd == "LASER_ON")
	{
printf("debug:laser_listener\n");
		system("roslaunch pkg_msgs laser_on.launch");
	}
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"node_laser_listener");
	ros::NodeHandle nodeHandle;
	ros::Subscriber subscriber = nodeHandle.subscribe("topic_server_cmd",1000,openLaser);
	ros::spin();
}
