#include <iostream>
#include "pkg_msgs/MsgOdometrySensor.h"
#include "SerialPort.h"

using namespace std;
const double deg2arc = M_PI/180;
int initAngle=0;

double getAngle(SerialPort* sp)
{
	double angle = 0;
	char start;
	char buffer[7];
	int endIdx;
	while(true)
	{
		endIdx = 0;
		while(sp->readPort(&start,1)<1){}
		if(start == '$')
		{
			while(sp->readPort(&start,1)<1 || start != ','){}
			while(true)
			{
				if(sp->readPort(&start,1)==1)
				{
					if(start == 'N')
					{
						endIdx -= 2;	//remove 'N' and ','	
						break;
					}
					else if(start == 'P'||start == 'O' || endIdx>=7)
					{
						endIdx=-1;
						break;
					}
					else
					{
						buffer[endIdx] = start;
						endIdx++;
					}
				}
			}
			if(endIdx > 0)
			{
				sp->flush();
				break;
			}
		}
	}
	//处理数据
	int intBit = endIdx-1;	//+1-2
	for(int i=0;i<intBit;i++)
	{
		angle += (buffer[i]-48) * pow(10,(intBit-1-i));
	}
	double dec=buffer[endIdx];
	angle += dec/10;
	angle = angle*deg2arc;		//弧度表示
/*	angle = initAngle-angle;		//转换为逆时针偏差角度
	if(angle<0)
	{
		angle += 2*M_PI;
	}
	*/
	return angle;
}

void calibrate(SerialPort *sp)
{
	cout << "calibrateing...\r\n";
	char calCmd[20] = "#F33.4=0*51\r\n";
	char saveCmd[20] = "#F2FE.2=1*67\r\n";
	char retCmd[20] = "#F33.4=1*50\r\n";
	sp->writePort(calCmd,strlen(calCmd)+1);
	sleep(30);
	sp->writePort(saveCmd,strlen(saveCmd)+1);
	usleep(50);	
	sp->writePort(retCmd,strlen(retCmd)+1);
	cout << "calibrated\r\n";
}

int main(int argc,char** argv)
{
	SerialPort compPort = *(new SerialPort("/dev/ttyUSB0"));
	compPort.openPort();
	compPort.setPort(19200,8,1,'n');
	compPort.setInMode('r');
	calibrate(&compPort);
	while(1)
	{
	/*	char tmp[101];
		int nBytes = compPort.readPort(tmp,100);
		tmp[nBytes] = '\0';
		cout << tmp << "\r\n";
		usleep(50);
		*/
		double angle = getAngle(&compPort);
		cout << "angle:" << angle/deg2arc << endl;
	}
}


