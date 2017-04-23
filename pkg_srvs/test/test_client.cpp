#include "ros/ros.h"
#include "pkg_srvs/SrvMode.h"

int main(int argc, char** argv)
{
	ros::init(argc,argv,"node_test_client");
	ros::NodeHandle nodeHandle;
	ros::ServiceClient testModeClient = nodeHandle.serviceClient<pkg_srvs::SrvMode>("srv_mode");
	pkg_srvs::SrvMode modeService;
	modeService.request.cmd = "";
	if(argc>1)
	{
		modeService.request.cmd = argv[1];
	}
	if(testModeClient.call(modeService))
	{
		ROS_INFO("modeService state:%d",modeService.response.state);
	}
	return 0;
}
