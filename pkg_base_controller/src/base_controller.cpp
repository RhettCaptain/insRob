#include <iostream>
#include <ros/ros.h>
#include "SerialPort.h"

int main(int argc, char** argv)
{
	SerialPort basePort = *(new SerialPort("/dev/ttyACM0"));
	basePort.openPort();
	basePort.setPort(115200,8,1,'n');
	while(1)
	{
		char sendBuff[10] = "e\r";
		char readBuff[10];
		int readBytes;
		basePort.writePort(sendBuff,2);
		readBytes = basePort.readPort(readBuff,10);
		cout << readBuff;
		
	}
}

