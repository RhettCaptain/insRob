#include "ros/ros.h"
#include "pkg_msgs/MsgServerCmd.h"


void openGmapping(const pkg_msgs::MsgServerCmd::ConstPtr& msg)
{
	if(msg->cmd == "MAP_ON")
	{
		system("roslaunch pkg_msgs map_on.launch");
	}
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"node_mapping_listener");
	ros::NodeHandle nodeHandle;
	ros::Subscriber subscriber = nodeHandle.subscribe("topic_server_cmd",1000,openGmapping);
	ros::spin();
}
