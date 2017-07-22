#include "ros/ros.h"
#include "pkg_msgs/MsgPointInfo.h"
#include "pkg_srvs/SrvReturn.h"
#include <stack>
#include <string>

using namespace std;

struct PointInfo
{
	string location;
	string speed;
	int pointlevel;
};

stack<PointInfo> routeStack;	


void record(const pkg_msgs::MsgPointInfo::ConstPtr& msg)
{
	PointInfo temp;  
	temp.location = msg->location;
	temp.speed = msg->speed;
	temp.pointlevel = msg->pointlevel;
	routeStack.push(temp);
}

bool returnRoute(pkg_srvs::SrvReturn::Request  &req, pkg_srvs::SrvReturn::Response &res)
{
	bool onMainRoad = false;
	if(routeStack.empty())
	{
		return false;
	}
	if(routeStack.top().pointlevel==1)
        {
                onMainRoad = true;
        }
        string localSeq = routeStack.top().location;
	string speedSeq = routeStack.top().speed;
	routeStack.pop();
	while(!routeStack.empty())
	{
		if(!onMainRoad)
		{
			if(routeStack.top().pointlevel==1)
			{
				onMainRoad = true;
			}
			localSeq += routeStack.top().location;
			speedSeq += routeStack.top().speed;
			routeStack.pop();
		}
		else
		{
			if(routeStack.top().pointlevel==1)
			{
				localSeq += routeStack.top().location;
				speedSeq += routeStack.top().speed;
			}
			routeStack.pop();
		}
	}
        res.location = localSeq;
	res.speed = speedSeq;
	return true;
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"node_return");
	ros::NodeHandle nodeHandle;
	ros::Subscriber sub = nodeHandle.subscribe("topic_point_info",10,record);
	ros::ServiceServer returnService = nodeHandle.advertiseService("srv_return",returnRoute);
	ros::spin();
	return 0;
}
