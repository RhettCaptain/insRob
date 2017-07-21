#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include "pkg_msgs/MsgOdometrySensor.h"
#include "communication/state.h"
using namespace std;
fstream out;

/*
void callback(const pkg_msgs::MsgOdometrySensor &msg)
{
	out<<msg.type;out<<" ";
	out<<msg.vlf; out<<" ";
	out<<endl;
}

*/

void callback(const communication::state &msg)
{
	out<<msg.type;out<<" ";
	out<<msg.lenth; out<<" ";
	out<<endl;
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"save_data");
	ros::NodeHandle nh;
	ros::Subscriber sub=nh.subscribe("communication/state_move",100,&callback);
	
	out.open("/home/user/temp.txt",ios::out|ios::app);
	printf("begin save data\n");
	while(ros::ok())
	{
		ros::spinOnce();
	}
	out.close();
	printf("save end\n");
	return 0;
}
