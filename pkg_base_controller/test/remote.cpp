#include <iostream>
#include <ros/ros.h>
#include "SerialPort.h"
#include <stdlib.h>
#include <stdio.h>
#include <cstring>
#include "pkg_msgs/MsgOdometrySensor.h"
#define TTY_PATH            "/dev/tty"
#define STTY_US             "stty raw -echo -F "
#define STTY_DEF            "stty -raw echo -F "
/*	#include "sensor_msgs/LaserScan.h"
	sensor_msgs::LaserScan scan_msg;

void copy(float* src,float* dst)
{
	int size = sizeof(src)/sizeof(float);
	for(int i=0;i<size;i++)
	{
		dst[i] = src[i];
	}
}
void debugScan(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	scan_msg.header = msg->header;
	scan_msg.angle_min = msg->angle_min;
	scan_msg.angle_max = msg->angle_max;
	scan_msg.angle_increment = msg->angle_increment;
	scan_msg.time_increment = msg->time_increment;
	scan_msg.scan_time = msg->scan_time;
	scan_msg.range_min = msg->range_min;
	scan_msg.range_max = msg->range_max;
	scan_msg.ranges = msg->ranges;
	scan_msg.intensities = msg->intensities;
//	copy(msg->ranges,scan_msg.ranges);
//	copy(msg->intensities,scan_msg.intensities);
}*/
static int get_char();

static int get_char()
{
    fd_set rfds;
    struct timeval tv;
    int ch = 0;

    FD_ZERO(&rfds);
    FD_SET(0, &rfds);
    tv.tv_sec = 0;
    tv.tv_usec = 10; //ÉèÖÃµÈ´ý³¬Ê±Ê±¼ä

    //¼ì²â¼üÅÌÊÇ·ñÓÐÊäÈë
    if (select(1, &rfds, NULL, NULL, &tv) > 0)
    {
        ch = getchar(); 
    }

    return ch;
}
int main(int argc, char** argv)
{
	ros::init(argc,argv,"node_base_controller");
	ros::NodeHandle nodeHandle;
	ros::Publisher odometrySensorPublisher = nodeHandle.advertise<pkg_msgs::MsgOdometrySensor>("topic_odometry_sensor",1000);
/*	
ros::Subscriber scanDebugSub = nodeHandle.subscribe("scan_debug",1000,debugScan);
ros::Publisher scanDebugPub = nodeHandle.advertise<sensor_msgs::LaserScan>("scan",10);
*/
	pkg_msgs::MsgOdometrySensor odometrySensorMsg;
	int ch = 0;
    	system(STTY_US TTY_PATH);
	SerialPort basePort = *(new SerialPort("/dev/ttyACM0"));
	basePort.openPort();
	basePort.setPort(115200,8,1,'n');
	basePort.setInMode('r');
	basePort.setOutMode('r');
	int lastLeftEncoder,lastRightEncoder;
	int curLeftEncoder,curRightEncoder;
	char sendBuff[10] = "r\r";
	basePort.writePort(sendBuff,2);
	lastLeftEncoder =0;
	lastRightEncoder =0;
	char moveForward[20] = "z 10 10\r";
	char moveBack[20] = "z -10 -10\r";
	char moveLeft[20] = "z -10 10\r";
	char moveRight[20] = "z 10 -10\r";
	while(ros::ok())
	{
		ch = get_char();
       	if (ch)
       	{
			switch(ch)
			{
			case 3://ctrl+c
                   	 {system(STTY_DEF TTY_PATH);return 0;}
			case 'w':
				basePort.writePort(moveForward,20);
				break;
			case 's':
				basePort.writePort(moveBack,20);
				break;
			case 'a':
				basePort.writePort(moveLeft,20);
				break;
			case 'd':
				basePort.writePort(moveRight,20);
				break;
			default:
				fflush(stdin);
				break;
			}
			usleep(50*1000);
			char dropBuff[100];
			basePort.readPort(dropBuff,100);
		}
		
		cout << "debug\r\n";
	  	sendBuff[0] = 'e';
		sendBuff[1] = '\r';
		
//		fflush(stdin);
//		usleep(50*1000);
		char readBuff[100];	
		int readBytes;
		basePort.flush();
		usleep(50*1000);
		basePort.writePort(sendBuff,2);
		usleep(50*1000);	
		readBytes = basePort.readPort(readBuff,100);
		if(readBytes>0  && (readBuff[0] == 'e'))
		{
			readBuff[readBytes]='\0';
	//		cout << "readBuff " << readBuff << endl;
			char chaTmp[20];
			int i = 0;
			while(readBuff[33+i]!='\0')
			{
				chaTmp[i] = readBuff[33+i];
				i++;
			}
			chaTmp[i] = '\0';
			if(i != 0)
			{
			curLeftEncoder = atoi(strtok(chaTmp," "));
			curRightEncoder = atoi(strtok(NULL," "));
			cout << curLeftEncoder << "-" << curRightEncoder<<"\r\n";
			double ratio = 0.00033;// * M_PI * 0.05 / 200; 
			odometrySensorMsg.vlf = (curLeftEncoder - lastLeftEncoder)*ratio;
			odometrySensorMsg.vlb = (curLeftEncoder - lastLeftEncoder)*ratio;
			odometrySensorMsg.vrf = (curRightEncoder - lastRightEncoder)*ratio;
			odometrySensorMsg.vrb = (curRightEncoder - lastRightEncoder)*ratio;
			odometrySensorMsg.type="ODOMETER";
			odometrySensorPublisher.publish(odometrySensorMsg);
			lastLeftEncoder = curLeftEncoder;
			lastRightEncoder = curRightEncoder;
//scanDebugPub.publish(scan_msg);
			}
		}
		sleep(1);
//		usleep(500*1000);
	}
}

