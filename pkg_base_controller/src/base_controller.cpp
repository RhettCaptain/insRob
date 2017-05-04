#include <iostream>
#include <ros/ros.h>
#include "SerialPort.h"
#include <stdlib.h>
#include <cstring>
#include "pkg_msgs/MsgOdometrySensor.h"

int main(int argc, char** argv)
{
	ros::init(argc,argv,"node_base_controller");
	ros::NodeHandle nodeHandle;
	ros::Publisher odometrySensorPublisher = nodeHandle.advertise<pkg_msgs::MsgOdometrySensor>("topic_odometry_sensor",1000);
	pkg_msgs::MsgOdometrySensor odometrySensorMsg;
	SerialPort basePort = *(new SerialPort("/dev/ttyACM0"));
	basePort.openPort();
	basePort.setPort(115200,8,1,'n');
	basePort.setInMode('r');
	basePort.setOutMode('r');
//	char tmp[3] = "r\r";
//	basePort.writePort(tmp,3);
//	basePort.setBlock(false);
	int lastLeftEncoder,lastRightEncoder;
	int curLeftEncoder,curRightEncoder;
	char sendBuff[10] = "r\r";
	basePort.writePort(sendBuff,2);
	lastLeftEncoder =0;
	lastRightEncoder =0;
	while(ros::ok())
	{
//		cout << "ai" << endl;
//		cin >> sendBuff;
	  	sendBuff[0] = 'e';
		sendBuff[1] = '\r';
		char readBuff[100];
		int readBytes;
		basePort.writePort(sendBuff,2);
		readBytes = basePort.readPort(readBuff,100);
/*		for(int i=0;i<readBytes;i++)
		{
			cout << readBuff[i];
		}
*/		if(readBytes>0 && (readBuff[0] == 'e'))
		{
			readBuff[readBytes]='\0';
		//	cout <<"get " <<readBytes<<"@"<<readBuff;//<<endl;
			char chaTmp[20];
			int i = 0;
			while(readBuff[33+i]!='\0')
			{
				chaTmp[i] = readBuff[33+i];
				i++;
			}
			chaTmp[i] = '\0';
//			cout << chaTmp << "chaTMp";
			curLeftEncoder = atoi(strtok(chaTmp," "));
			curRightEncoder = atoi(strtok(NULL," "));
			cout << curLeftEncoder << "-" << curRightEncoder<<endl;
			double ratio = 2 * M_PI * 0.05 / 200; 
			odometrySensorMsg.vlf = (curLeftEncoder - lastLeftEncoder)*ratio;
			odometrySensorMsg.vlb = (curLeftEncoder - lastLeftEncoder)*ratio;
			odometrySensorMsg.vrf = (curRightEncoder - lastRightEncoder)*ratio;
			odometrySensorMsg.vrb = (curRightEncoder - lastRightEncoder)*ratio;
			odometrySensorMsg.type="ODOMETER";
			odometrySensorPublisher.publish(odometrySensorMsg);
			lastLeftEncoder = curLeftEncoder;
			lastRightEncoder = curRightEncoder;
		}
//		delay(500);		
//		sleep(1);
		usleep(900*1000);
	}
}

