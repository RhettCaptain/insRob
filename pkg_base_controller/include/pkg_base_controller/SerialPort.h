#ifndef SERIALPORT_H
#define SERIALPORT_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

typedef unsigned char uchar;

class SerialPort
{
public:
	int fd;		//device
	const char* port;	//portName
		
public:
	SerialPort(const char* portName);
	
	//
	//return 0(success) or -1(error)
	//
	int openPort();
		
	//
	//close opened port
	//
	int closePort();
	
	//
	//@baud:baudrate,9600,19200,115200
	//@databits:7 or 8
	//@stopbits:1 or 2
	//@parity: 'n','N':none parity;'o','O':odd parity;'e','E':even parity;'s','S':space parity
	//return 0(success) or -1(error)
	//
	int setPort(int baud,int databits,int stopbits,char parity);

	//
	//@sendCon context to be sent
	//@len:	send length 
	//return counts of sent bytes
	int sendPort(char* sendCon,int len);

	//
	//@readCon buffer for read Context
	//@len:read length
	//return counts of read bytes
	int readPort(char* readCon,int len);
	

	
	//
	//@*p:pointer of uchar array to be checked
	//@len:lenth of array to be checked
	//return check num
	//
	uint crc16_modbus(uchar *p, int len);
	
	//
	//@on: 1/true:block mode
	//		0/false:nonblock mode
	//
	void setBlock(bool on);

private:


};


#endif
