#include <iostream>
#include <ros/ros.h>
#include "SerialPort.h"
#include "pkg_msgs/MsgOdometrySensor.h"

int main(int argc, char** argv)
{
	ros::init(argc,argv,"node_base_controller");
	ros::NodeHandle nodeHandle;
	ros::Publisher odometrySensorPublisher = nodeHandle.advertise<pkg_msgs::MsgOdometrySensor>("topic_odometry_sensor",1000);
	pkg_msgs::MsgOdometrySensor odometrySensorMsg;
	const double ratio = 0.00066;// * M_PI * 0.05 / 200; //编码器+1距离 
	while(ros::ok())
	{
		odometrySensorMsg.vlf = 0.03;
		odometrySensorMsg.vlb = 0.03;
		odometrySensorMsg.vrf = -0.03;
		odometrySensorMsg.vrb = -0.03;
		odometrySensorMsg.type="ODOMETER";
		odometrySensorPublisher.publish(odometrySensorMsg);
		usleep(100*1000);
	}
}

