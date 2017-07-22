/*通信模块:与上位机通信
  实现功能:1.接受上位机指令,进行校验回复正常接受与否.
		  2.发布指令话题"communication/cmd"
		  3.订阅状态信息,当上位机查询时,发送状态信息		  
*/
#include <communication/udp_server.h>
#include <communication/display.h>
#include <communication/state.h>

#define  CONNECT_CMD 	        0x00                                 //连接指令
#define  ASK_CMD      		0x50  	                            //查询指令
#define  SET_CMD		0x10			          //peizhi指令		     
#define  BATTERY_CMD            0xA0                              //电源数据

bool UDP_Server::open()
{
 	if((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) 
    	{
       		perror("Creatingsocket failed.");
       		// exit(1);
       		return false;
    	}
	else
	{
		printf("open ok\n");
		return true;
	}
}

/* 函数名:init
 *入口参数:端口号,发送数组,接受数组,以及ros节点句柄	
 *功能:初始化服务端,绑定端口,初始化数组以及订阅器和发布器
*/                                           
void UDP_Server::init(int port,char *send,char *rec,ros::NodeHandle &n,
	int wrong_time,int broken_time)
{
	broken=false;	
   	connect=false;
   	bzero(&server,sizeof(server));  
    	server.sin_family=AF_INET;		
   	server.sin_port=htons(port);	
    	server.sin_addr.s_addr= htonl (INADDR_ANY);
	if(bind(sockfd, (struct sockaddr *)&server, sizeof(server)) == -1)
	{
	       perror("Bind()error.");
	      // exit(1);  
	  }   
	fcntl(sockfd,F_SETFL,O_NONBLOCK);
	sendBuf=send;
	recBuf=rec;
	count=0;
	sub_state=n.subscribe("processor/state",100,&UDP_Server::callback,this);
	sub_state2=n.subscribe("/communication/state_battery",2,&UDP_Server::battery_callback,this);
	pub_command=n.advertise<communication::command>("communication/cmd",10);
	time_broken=broken_time;
	time_wrong =wrong_time;
}

/*函数名:callback
 *功能:回调函数	
*/
void UDP_Server::callback(const communication::state &msg)
{
	data_out="";
	data_out+=msg.data;
}
void UDP_Server::battery_callback(const communication::state &msg)
{
	data_battery="";
	data_battery+=msg.data;
}

/* 函数名:wait_connect  等待连接
 *入口参数:指令最大字节数,以及通信等待时间.
 *功能: 等待连接直至连接成功,成功发布连接成功消息
*/   

void UDP_Server::wait_connect(int maxsize,int time)
{
	fd_set fds;
	timeval timeout={time,0};
	int net;
	
	while(!connect)
	{
		timeout.tv_sec=time;
		timeout.tv_usec=0;
		cout<<endl;
		printf("state : connecting\n");
		FD_ZERO(&fds);
		FD_SET(sockfd,&fds);
		net=0;
		net=select(sockfd+1,&fds,NULL,NULL,&timeout);
	
		if(net<0)
		{
			exit(-1);
		}
		else if(net==0) 
		{
			printf("timeout\n");

		}
				
		 else
		 {	if(FD_ISSET(sockfd,&fds))
				{
					receive(maxsize);
					if(connect)
					{
						printf("construct connect\n");
						pub_msg.type=type;
						pub_msg.command_lenth=cmd_lenth;
						pub_msg.data=data_in;
						pub_msg.break_flag=false;
						pub_msg.wrong_flag=!connect;
						pub_command.publish(pub_msg);
					}
				}
				
		 }
	
	}
	
}

/* 函数名:wait_command  等待指令
 *入口参数:指令最大字节数,以及通信等待时间.
 *功能: 等待指令,等待时间内到来则正常回复,并发送命令消息
 		超时则终止,并发布通信中断
*/ 
void UDP_Server::wait_command(int maxsize)
{
	fd_set fds;
	timeval timeout={time_wrong,0};
	int net;
	connect=true;
	
	while(1)
	{
		cout<<endl;
		printf("state : connected\n");
	   	 timeout.tv_sec=time_wrong;
		timeout.tv_usec=0;

		FD_ZERO(&fds);
		FD_SET(sockfd,&fds);
		net=0;
		net=select(sockfd+1,&fds,NULL,NULL,&timeout);
		
		if(net<0)
		{
			exit(-1);
		}
		else if(net==0) 
		{
			printf("%ds  timeout\n",time_wrong);
			pub_msg.wrong_flag=true;
			pub_msg.break_flag=false;
			pub_command.publish(pub_msg);
			connect=false;	
			break;
		}
				
		 else
		 {	if(FD_ISSET(sockfd,&fds))
				{
					receive(maxsize);
					pub_msg.type=type;
					pub_msg.command_lenth=cmd_lenth;
					pub_msg.data=data_in;
					pub_msg.break_flag=false;
					pub_msg.wrong_flag=false;
					
					pub_command.publish(pub_msg);
					
				}
				
		 }
	
		ros::spinOnce();
	}
}

/* 函数名:wait_reconnect  等待重连
 *入口参数:指令最大字节数,以及通信等待时间.
 *功能: 等待指令,等待时间内到来则正常回复,并发送命令消息,表示重连成功
 		超时则终止,并发布通信故障
*/ 
void UDP_Server::wait_reconnect(int maxsize)
{
	fd_set fds;
	timeval timeout={time_broken,0};
	int net;
	
	while(!connect)
	{
		cout<<endl;
		timeout.tv_sec=time_broken;
	  	timeout.tv_usec=0;
		printf("state : connecting\n");
		FD_ZERO(&fds);
		FD_SET(sockfd,&fds);
		net=0;
		net=select(sockfd+1,&fds,NULL,NULL,&timeout);
	
		if(net<0)
		{
			exit(-1);
		}
		else if(net==0) 
		{
			printf("%ds  timeout\n",time_broken);
			pub_msg.wrong_flag=true;
			pub_msg.break_flag=true;
			pub_command.publish(pub_msg);
			broken=true;
			break;
		}
				
		 else
		 {	if(FD_ISSET(sockfd,&fds))
				{
					receive(maxsize);
					pub_msg.type=type;
					pub_msg.command_lenth=cmd_lenth;
					pub_msg.data=data_in;
					pub_msg.break_flag=false;
					pub_msg.wrong_flag=!connect;
					pub_command.publish(pub_msg);

				}
				
		 }
	
	}
	
}

/* 函数名:receive  接受数据
 *入口参数:指令最大字节数
 *功能: 接受数据
*/ 
void UDP_Server::receive(int max)
{	count++;
	socklen_t addrlen;
	addrlen=sizeof(server);	
	
	int num=0;
	bzero(recBuf, sizeof(recBuf));
	
	num =recvfrom(sockfd,recBuf,max,0,(struct sockaddr*)&server,&addrlen); 
  	cout<<"rece message "<<count<<": ";
  	HexDump(recBuf,num,0);
  	if(num>0)
  	{
		process();
		
	}	
	
}


/* 函数名:process  数据处理
 *功能: 处理数据,校验上位机数据.产生回复指令
 	    发送上位机指令给处理器
 	    若为连接指令.则connect=true;
*/ 
void UDP_Server::process()
{

	bzero(sendBuf, sizeof(sendBuf));

	int len=strlen(recBuf);
	socklen_t addrlen;
	addrlen=sizeof(server);	
	
	char *p=recBuf;
	string out="";
	
	unsigned char head=*p++;
	unsigned char order=*p++;
	string ID="";
	string data="";
	string data_crc="";

	ID+=*p++;ID+=*p++;ID+=*p++;ID+=*p++;

	// lenth
	unsigned char lenth[]={0,0};
	lenth[0]=*p++;
	lenth[1]=*p++;
	
	//for processor
	cmd_lenth=0;
	cmd_lenth=(int)lenth[0]*256+(int)lenth[1];
	

	unsigned char t=*p++;

	for(int i=0;i<cmd_lenth-1;i++)
	{
		data+=*p++;
	}
	
	unsigned char check=*p++;
	unsigned char tail=*p;
	
	unsigned char check_out;//crc
	unsigned char check_in;//crc
	unsigned char rec=0x01; //receive normal   as data
	
	//test 
	//


	out+=head;
	out+=order;
	out+=ID;
	string len_out;
	len_out[0]=0x00;
	len_out[1]=0x02;
	
	data_crc+=head;data_crc+=order;data_crc+=ID;data_crc+=lenth[0];data_crc+=lenth[1];
	data_crc+=t;data_crc+=data;

	/*char *buf;
	buf=(char *)data_crc.c_str();
	int out_l=data_crc.size();
	cout<<"send message :";
  	HexDump(buf,out_l,0);
*/
	if(tail==0xFF)
	{
		
		check_in=getCrc(data_crc);
		//check_in=0x0A;  // **if you want test  ,use this sentence
		if(check==check_in)
		{	
			type="";
			data_in="";
			// for processor
			type+=t;
			data_in=data;	
			char *buf1;
			buf1=(char *)type.c_str();
			int out_2=type.size();
			cout<<"type :";
  			HexDump(buf1,out_2,0);

			char *buf;
			buf=(char *)data_in.c_str();
		             int out_l=data_in.size();
	                          cout<<"data :";
  	                          HexDump(buf,out_l,0);
			//data
			if(t==ASK_CMD)  // ask for state information
			{
			   	printf("ask command\n");
			   	 len_out[0]=0x00;
			   	 len_out[1]=data_out.size()+1;

				out=out+len_out[0];
				out=out+len_out[1];
                                printf("the lenth is %d\n",(int)len_out[1]);
                                
				out+=t;
				//out+=rec;
				out+=data_out;
				check_out=getCrc(out);// update crc		
			}
			else if(t==CONNECT_CMD)
			{
				printf("connect command\n");
				out=out+len_out[0];
				out=out+len_out[1];	
				out+=t;
				out+=rec;
				check_out=getCrc(out);
				connect=true;
			}
			else if(t==SET_CMD)
			{
				printf("set command\n");
				out=out+len_out[0];
				out=out+len_out[1];

				out+=t;
				out+=rec;
				check_out=getCrc(out);
				time_wrong=data[5];
				time_broken=data[7];
				//printf("wrong : %d, broken : %d \n",time_wrong,time_broken );

			}
			else if(t==BATTERY_CMD)
			{
			        printf("battery command\n");
			   	int temp_lenth=data_battery.size();
			   	if(temp_lenth>128)
			   	{ len_out[0]=0x01;
			   	  len_out[1]=temp_lenth-128;
                                 }
                                 else
                                 {
                                  len_out[0]=0x00;
			   	  len_out[1]=temp_lenth;
                                 }
				out=out+len_out[0];
				out=out+len_out[1];
                                printf("the lenth is %d\n",(int)len_out[1]);
                                
				out+=t;
				//out+=rec;
				out+=data_battery;
				check_out=getCrc(out);// update crc	

			}
			else 
			{ 
				printf("other command\n");
				out=out+len_out[0];
				out=out+len_out[1];

				out+=t;
				out+=rec;
				check_out=getCrc(out);

				//get the time

			}
		}
		else// check failed ,return receive error
		{	
			out=out+len_out[0];
			out=out+len_out[1];

			out+=t;
			rec=0x00;
			out+=rec;
			check_out=getCrc(out);// update crc
		}
	}
	
	else //the command not compelete 
	{	
		out=out+len_out[0];
		out=out+len_out[1];

		out+=t;
		rec=0x00;
		out+=rec;
		check_out=getCrc(out);// default check_out 
	}
	
   //check_out=0x0A;	
	out+=check_out;
	out+=tail;
 	
	sendBuf=(char *)out.c_str();
	
	int out_len=out.size();
	
	cout<<"send message :";
  	HexDump(sendBuf,out_len,0);

	int c=(int)check_out;
	cout<<"crc  is "<<c<<endl;
	
	if(connect)
	{
		sendto(sockfd,sendBuf, out_len, 0, (struct sockaddr *)&server, addrlen);
	}
}
/*函数名:getCrc
  功能:产生校验
*/
unsigned char UDP_Server::getCrc(string values)
{
	unsigned char crc_array[256] = {
		0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83,
		0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
		0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e,
		0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
		0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0,
		0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
		0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d,
		0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
		0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5,
		0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
		0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58,
		0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
		0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6,
		0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
		0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b,
		0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
		0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f,
		0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
		0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92,
		0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
		0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c,
		0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
		0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1,
		0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
		0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49,
		0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
		0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4,
		0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
		0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a,
		0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
		0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7,
		0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
	};
	unsigned char crc8 = 0;
	for (int i = 0; i < values.size(); i++)
	{
		crc8 = crc_array[crc8^(unsigned char)(values[i])];
	}
	return crc8;
}


void UDP_Server::close_ser()
{
	close(sockfd);  	
}


