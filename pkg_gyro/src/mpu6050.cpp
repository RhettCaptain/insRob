#include <iostream>
#include "SerialPort.h"

using namespace std;

double getAngleSpd(SerialPort* sp)
{
	char start=0;
	bool ok=false;
	double angleSpd = 0;
	while(!ok)
	{
		while(start!=0x55)
		{
			sp->readPort(&start,1);
		}
		sp->readPort(&start,1);
		if(start == 0x52)
		{
			char buff[9];
			if(sp->readPort(buff,9)==9)
			{
				unsigned char sum=0x55+0x52;
				for(int i=0;i<8;i++)
				{
					sum+=(unsigned char)buff[i];
				}
				if(sum != (unsigned char)buff[8])
				{
					cout << "check fail!\r\n";
					continue;
				}
				int highByte = buff[5];
				char lowByte = buff[4];
				angleSpd = (highByte << 8 | lowByte)/32768.0*2000;
				ok = true;
			}

		}
	}
	return angleSpd;
	
}
int main(int argc, char** argv)
{
	SerialPort mpuPort = *(new SerialPort("/dev/ttyUSB0"));
	mpuPort.openPort();
	mpuPort.setPort(115200,8,1,'n');
	mpuPort.setInMode('r');
	char read;
	while(1)
	{
		double angleSpd = getAngleSpd(&mpuPort);
		cout << "angleSpd: " << angleSpd << "\r\n";
	}
}

