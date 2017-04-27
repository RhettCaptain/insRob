#include "SerialPort.h"
#include <ros/ros.h>
#include <iostream>

using namespace std;

int main(int argc, char** argv)
{
	char* dev;

	ros::init(argc,argv,"node_remote");
	ros::NodeHandle nodeHandle;
	dev = "/dev/ttyACM0";	
//	nodeHandle.param("dev",dev,"/dev/ttyACM0");
SerialPort* portMotor = new SerialPort(dev);
	portMotor->openPort();
	portMotor->setPort(115200,8,1,'n');
	char test[100];
	int nBytes;
for(int i=0;i<100;i++){	
	nBytes = portMotor->sendPort("e\r",3);
	if(nBytes>0){
	cout << "send " << nBytes <<endl;
	portMotor->readPort(test,100);
	cout << "test1 " << test[0] << endl;
	cout << "read " << test << endl;
}		}
}
