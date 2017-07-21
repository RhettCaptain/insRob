#ifndef UDP_SERVER_H
#define UDP_SERVER_H

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

#include <communication/state.h>
using namespace std;

class UDP_Server
{

	public:
	//open udp
	bool open();
	/* initial the socket server;
		parameter: PORT
	*/
	void init(int port,char *send,char *rec,ros::NodeHandle &n,
		int wrong_time,int broken_time);
	void process();
	void receive(int max);

	void close_ser();
	void callback();
	unsigned char getCrc(string values);
	void wait_command(int maxsize);
	void wait_connect(int maxsize,int time);
	void wait_reconnect(int maxsize);
	void callback(const communication::state &msg);

	private:
	int sockfd;
	char *sendBuf;	
	char *recBuf;
	struct sockaddr_in server;
	int count;

	
	public:
	int cmd_lenth;
	string type;
	string data_in;
	string data_out;
	
	communication::command pub_msg;
	ros::Subscriber sub_state;
	ros::Publisher pub_command;
	bool connect;
	bool broken;
	int  time_wrong;
	int time_broken;
	
};

#endif
