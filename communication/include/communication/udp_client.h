#ifndef UDP_CLIENT_H
#define UDP_CLIENT_H

#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <fcntl.h>

#include <cstring>
#include <string>
#include <iostream>
#include <ros/ros.h>
#include <communication/command.h>
#include <sys/select.h>
using namespace std;

class UDP_Client
{

	public:
	//open udp
	bool open();
	/* initial the socket server;
		parameter: PORT
	*/
	void init(int port,string ip,char *rec,string ID);
	void sendInfo(string type ,string data);

	void receive(int max);
	void close_ser();
	
	unsigned char getCrc(string values);
	void wait_back(int maxsize,int time);


	private:
	int sockfd;

	char *recBuf;
	struct sockaddr_in client;
	unsigned char order; //zhen xuhao
	bool connFlag;
	string dev_ID;



	public:
	string data_in;
	string type;
	
	bool receFlag;
	bool connect;

	int len;
	ros::Publisher pub_command;	
};

#endif
