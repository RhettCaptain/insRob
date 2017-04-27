#ifndef SERIALPORT_H
#define SERIALPORT_H

#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

using namespace std;

class SerialPort
{
private: 
	int fd;		
	const char* portName;	
	bool openState;
	bool blockState;
	float outTime;
	int baud;
	int dataBits;
	int stopBits;
	char parity;
	char inMode;
	char outMode;
	struct termios options;
public:
	SerialPort(const char* pPortName);
	~SerialPort();
	
	int openPort();
	int openPort(int flag);
	int openPort(int flag,int perms);
	void closePort();
	
	void setBaud(int pBaud);
	int getBaud();
	void setDataBits(int pDataBits);
	int getDataBits();
	void setStopBits(int pStopBits);
	int getStopBits();
	void setParity(char pParity);
	char getParity();
	void setPort();
	void setPort(int pBaud,int pDataBits,int pStopBits,char pParity);
	
	bool isOpen();
	bool isBlock();
	void setBlock(bool on);
	void setOutTime(float pOutTime);
	float getOutTime();

	//@mode:'c'-classic mode
	//     'r'-raw mode
	void setInMode(char mode);
	char getInMode();
	void setOutMode(char mode);
	char getOutMode();

	int readPort(char* buffer,int len);
	int writePort(char* buffer,int len);
};


#endif
