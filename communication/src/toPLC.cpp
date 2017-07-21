/*通信模块:与PLC或运动卡通信
  实现功能:1.接受主控的指令,生成指令发送给下位机(PLC或运动卡)
		  2.接收下位机反馈指令,进行校验,并回复给上位机,通知上位机接受正确与否
说明:receFlag:表示接受消息状态 
	 connect:表示通信子模块连接状态	
	 举例说明:若receFlat=false,则需要重新发送该指令. 一是超时,二是数据错误
	 		  若connnect=false,则需要发送重连指令 
*/

#include <communication/udp_client.h>
#include <communication/display.h>


/*  crc for test  ,when use in normal ,delete the crc=0x10
*/
bool UDP_Client::open()
{
 	if((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) 
    	{
       		perror("Creatingsocket failed.");
       		 exit(1);
       		return false;
    	}
	else
	{
		printf("open ok\n");
		return true;
	}
}

/* 函数名:init
 *入口参数:端口号,发送数组,接受数组,以及ros节点句柄,设备ID	
 *功能:初始化服务端,绑定端口,初始化数组以及连接状态和数据状态
*/   
void UDP_Client::init(int port,string ip,char *rec,string ID)
{
	
   	bzero(&client,sizeof(client));  
    	client.sin_family=AF_INET;		
    	client.sin_port=htons(port);	
    	client.sin_addr.s_addr= inet_addr(ip.c_str());
    	//printf("%s\n",ip.c_str() );
    	order=0x00;
    	
    	receFlag=false;
    	connect=false;
    	dev_ID=ID;
    	recBuf=rec;
}


/* 函数名:sendInfo
 *入口参数:指令类型,以及指令数据
 *功能:生成指令发送给下位机
*/ 
void UDP_Client::sendInfo(string type ,string data)
{
	socklen_t addrlen;
//	socklen_t addr_len = sizeof(struct sockaddr_in);		

	string cmdToPlc="";
	char *sendBuf;	

	//tranform into the standard command to plc
	/*unsigned char head=0x41;
	order=0x42;
	string ID=ROBOT_ID; 
*/
 	unsigned char  lenth1;
  	unsigned char lenth2; 	

	unsigned char head=0xFA;
	if(order==255){order=0x00;}else order+=1;
	

	    int temp=data.size()+1; //zhiling changdu
	    if(temp>255)  
	    {
	    	lenth1=temp-255;
	        	lenth2=255;
	    }
	    else 
	    {
	        lenth1=0;
	        lenth2=temp;
	    }



    cmdToPlc+=head;cmdToPlc+=order;cmdToPlc+=dev_ID;cmdToPlc+=lenth1;cmdToPlc+=lenth2;
		
	cmdToPlc+=type;
	cmdToPlc+=data;


	unsigned char crc=getCrc(cmdToPlc);
	unsigned tail=0xFF;
	//test
	//crc=0x10;

	cmdToPlc+=crc;
	cmdToPlc+=tail;

	sendBuf=(char *)cmdToPlc.c_str();
	int out_len=cmdToPlc.size();
	
	cout<<"send messaage: ";
	HexDump(sendBuf,out_len,0);
	//printf("send out %s\n",cmdToPlc.c_str());
	sendto(sockfd,sendBuf, out_len, 0, (struct sockaddr *)&client, sizeof(client));
	
   	receFlag=false;
}


/* 函数名:wait_back
 *入口参数:最大字节长度,通信时间
 *功能:等待下位机回复
*/ 
void UDP_Client::wait_back(int maxsize,int time)
{
	fd_set fds;
	timeval timeout={time,0};
	int net;
	
	while(!receFlag)
	{
		timeout.tv_sec=time;
		timeout.tv_usec=0;
		
		FD_ZERO(&fds);
		FD_SET(sockfd,&fds);
		net=0;
		net=select(sockfd+1,&fds,NULL,NULL,&timeout);
	
		if(net<0)
		{
			exit(-1);
			printf("error\n");
		}
		else if(net==0) 
		{
			printf("recevie timeout\n");
			receFlag=false;       
			connect=false;     // timeout means connect failed 
			break;
		}	
		 else
		 {	if(FD_ISSET(sockfd,&fds))
			{
				connect=true;
				receive(maxsize);		
			}
		 }
	
	}
}


/* 函数名:receive  接受数据
 *入口参数:指令最大字节数
 *功能: 接受数据
*/ 
void UDP_Client::receive(int max)
{
	socklen_t addrlen;
	addrlen=sizeof(client);	
	len=0;
	int num=0;
	bzero(recBuf, sizeof(recBuf));
	num =recvfrom(sockfd,recBuf,max,0,(struct sockaddr*)&client,&addrlen); 
	len=num;
	
	cout<<"rece messaage: ";
	HexDump(recBuf,len,0);
	
	char *p=recBuf;
	

	//printf("back lenth%d\n",num );
	string crc_data="";
	data_in="";
	unsigned char crc_get;

	for (int i=0;i<num-1;i++)
	{
		if(i<num-2)
	            	 {	crc_data+=*(p+i);
	             }
	             else 
	              {
	       	       	crc_get=*(p+i);
	              }

	              if(5<i&&i<num-2)
	              {
	           	   	data_in+=*(p+i);
	              }
	}
	
	if(crc_get==getCrc(crc_data))
	//if(crc_get==0xA0)
	{
		printf("data is right \n");
		receFlag=true;
		//******//
		char *buf;
		buf=(char *)data_in.c_str();
		int out_len=data_in.size();
		cout<<"message to processor : ";
		HexDump(buf,out_len,0);
		//****//
	}
	else receFlag=false;
}


unsigned char UDP_Client::getCrc(string values)
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

void UDP_Client::close_ser()
{
	close(sockfd);  	
}
