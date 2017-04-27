#include "SerialPort.h"

SerialPort::SerialPort(const char* portName)
{
	port = portName;
}

int SerialPort::openPort()
{
	fd = open(port,O_RDWR | O_NOCTTY | O_NDELAY);
	if(-1 == fd)
	{
		perror("can't open device!\n");
		return(-1);
	}
	return 0;
}

int SerialPort::closePort()
{
	if(-1 == fd)
	{
		perror("no open device!\n");
		return(-1);
	}
	close(fd);
	return 0;
}

int SerialPort::sendPort(char* sendCon, int len)
{
	int nBytes;
	nBytes = write(fd,sendCon,len);
	return nBytes;
}

int SerialPort::readPort(char* readCon, int len)
{
	int nBytes;
	nBytes = read(fd, readCon, len);
	return nBytes;
}

int SerialPort::setPort(int baud,int databits,int stopbits,char parity)
{
	struct termios options;
	if(tcgetattr(fd,&options)!=0)
	{
		perror("can't get current option\n");
		return(-1);
	}
	switch(baud)
	{
	case 9600:
		cfsetispeed(&options,B9600);
		cfsetospeed(&options,B9600);
		break;
	case 19200:
		cfsetispeed(&options,B19200);
		cfsetospeed(&options,B19200);
		break;
	case 115200:
		cfsetispeed(&options,B115200);
		cfsetospeed(&options,B115200);
		break;
	default:
		break;	
	}
	options.c_cflag |= (CLOCAL|CREAD);
	
	options.c_cflag &= ~CSIZE;
	switch(databits)
	{
	case 7:
		options.c_cflag |= CS7;
		break;
	case 8:
		options.c_cflag |= CS8;
		break;
	default:
		fprintf(stderr,"Unsupported data size\n");
		return(-1);
	} 
	
	switch(parity)
	{
	case 'n':
	case 'N':
		options.c_cflag &= ~PARENB;
		options.c_iflag &= ~INPCK;
		break;
	case 'o':
	case 'O':
		options.c_cflag |= (PARODD | PARENB);
		options.c_iflag |= INPCK;
		break;
	case 'e':
	case 'E':
		options.c_cflag |= PARENB;
		options.c_cflag &= ~PARODD;
		options.c_iflag |= INPCK;
		break;
	case 's':
	case 'S':
		options.c_cflag &= ~PARENB;
		options.c_cflag &= ~CSTOPB;
		break;
	default:
		fprintf(stderr,"Unsipported parity\n");
		return(-1);
	}

	switch(stopbits)
	{
	case 1:
		options.c_cflag &= ~CSTOPB;
		break;
	case 2:
		options.c_cflag |= CSTOPB;
		break;
	default:
		fprintf(stderr,"Unsupported stop bits\n");
		return(-1);
	}

	if(parity != 'n')
	{
		options.c_iflag |= INPCK;
	}
	tcflush(fd,TCIOFLUSH);
	options.c_lflag &= ~(ICANON | ECHO | ECHOE);		//raw mode:input immediately, no echo
	options.c_oflag &= ~OPOST;		//output immediately without buff
	options.c_cc[VTIME]=50;		//mostly wait 5s for reading in block mode
	options.c_cc[VMIN]=0;
	if(tcsetattr(fd,TCSANOW,&options)!=0)
	{
		perror("can't set new options\n");
		return(-1);
	}
	return(0);
}

void SerialPort::setBlock(bool on)
{
	int flags;
	if(on)
	{
		flags = fcntl(fd,F_GETFL,0);
		flags &= ~O_NONBLOCK;
		fcntl(fd,F_SETFL,flags);
	}
	else
	{
		flags = fcntl(fd,F_GETFL,0);
		flags |= O_NONBLOCK;
		fcntl(fd,F_SETFL,flags);
	}
}

uint SerialPort::crc16_modbus(uchar *p, int len)
{
	char i;
	int j;
	uint crc=0xffff;
	
	for(j=0;j<len;j++)
	{
		crc^=(*p);
		p++;
		for(i=8;i!=0;i--)
		{
			if(crc&1)
			{
				crc>>=1;
				crc^=0xa001;
			}
			else
			{
				crc>>=1;
			}
		}
	}
	return crc;
}
