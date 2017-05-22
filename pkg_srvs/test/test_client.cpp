#include "ros/ros.h"
#include "pkg_srvs/SrvMode.h"
#include "pkg_srvs/SrvGetLine.h"
#include "pkg_srvs/SrvPointLine.h"
#include "pkg_srvs/SrvGetDistance.h"
#include "pkg_srvs/SrvGetYawBias.h"
#include <tf/transform_broadcaster.h>

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
	//test getline
	/*
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
	*/
	//test pointline
	/*
	ros::ServiceClient testPointLineClient = nodeHandle.serviceClient<pkg_srvs::SrvPointLine>("srv_point_line");
	pkg_srvs::SrvPointLine pointLineService;
	geometry_msgs::PoseWithCovariance pose;
	double line[3] = {0,1,0};
	pose.pose.position.x = 1;
	pose.pose.position.y = 1;
	pointLineService.request.pose = pose;
	pointLineService.request.line[0] = line[0];
	pointLineService.request.line[1] = line[1];
	pointLineService.request.line[2] = line[2];
	if(testPointLineClient.call(pointLineService))
	{
		ROS_INFO("pointLineService distance:%f",pointLineService.response.distance);
	}
	*/
	//test getYawBias
	/*
        ros::ServiceClient testGetYawBiasClient = nodeHandle.serviceClient<pkg_srvs::SrvGetYawBias>("srv_get_yaw_bias");
	pkg_srvs::SrvGetYawBias getYawBiasService;
	geometry_msgs::PoseWithCovariance pose;
	double line[3] = {-1,1,0};
	pose.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI/2);;
	getYawBiasService.request.pose = pose;
	getYawBiasService.request.line[0] = line[0];
	getYawBiasService.request.line[1] = line[1];
	getYawBiasService.request.line[2] = line[2];
	if(testGetYawBiasClient.call(getYawBiasService))
	{
		ROS_INFO("getYawBiasService theta:%f",getYawBiasService.response.theta);
	}
	*/
	//test getDistance
	
        ros::ServiceClient testGetDistanceClient = nodeHandle.serviceClient<pkg_srvs::SrvGetDistance>("srv_get_distance");
	pkg_srvs::SrvGetDistance getDistanceService;
	geometry_msgs::PoseWithCovariance poseA,poseB;
	poseA.pose.position.x = 4;
	poseA.pose.position.y = 0;
        poseB.pose.position.x = 0;
	poseB.pose.position.y = 3;
	getDistanceService.request.poseA = poseA;
	getDistanceService.request.poseB = poseB;
	if(testGetDistanceClient.call(getDistanceService))
	{
		ROS_INFO("getDistanceService distance:%f",getDistanceService.response.distance);
	}
	

	return 0;
}
