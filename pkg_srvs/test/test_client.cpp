#include "ros/ros.h"
#include "pkg_srvs/SrvMode.h"
#include "pkg_srvs/SrvGetLine.h"

int main(int argc, char** argv)
{
	ros::init(argc,argv,"node_test_client");
	ros::NodeHandle nodeHandle;
	//test mode
	/* 
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
	*/
	ros::ServiceClient testGetLineClient = nodeHandle.serviceClient<pkg_srvs::SrvGetLine>("srv_get_line");
	pkg_srvs::SrvGetLine getLineService;
	geometry_msgs::PoseWithCovariance poseA,poseB;
	poseA.pose.position.x = 1;
	poseA.pose.position.y = 1;
	getLineService.request.poseA = poseA;
	getLineService.request.poseB = poseB;
	if(testGetLineClient.call(getLineService))
	{
		ROS_INFO("getLineService lines[3]:{%f,%f,%f}",getLineService.response.line[0],getLineService.response.line[1],getLineService.response.line[2]);
	}
	return 0;
}
