/*通信模块: 与运动卡通信节点
 节点名: command_to_move
 订阅话题:无
 发布话题:communication/state_move
 服务:send_move_cmd
 功能:1.若未连接,则发送连接指令
 	  2.定时发布查询指令,并发布运动状态消息
 	  3.接受主控服务调用,发送所要求的指令
*/


#include <ros/ros.h>
#include <communication/udp_client.h>
#include <communication/sendCmd.h>
#include <communication/state.h>

#define	UDP_TEST_PORT		1030
#define UDP_SERVER_IP 		"192.168.0.252"// this should be changed
#define MAXSIZE 		1000
#define TIME_WRONG 		1//10s

 char MOVE_ID=0x01; 
 char CONNECT_TYPE=0x00;
 char CONNECT_DATA =0X01;
 char STATE_TYPE = 0X20;
 char STATE_DATA = 0X01;


bool sendCallback(communication::sendCmd::Request  &req,
         communication::sendCmd::Response &res) ; 

UDP_Client client;
int main(int argc,char ** argv)
{
	ros::init(argc,argv,"command_to_move");
	ros::NodeHandle nh;
	ros::ServiceServer send_Cmd = nh.advertiseService("send_move_cmd", sendCallback);
	ros::Publisher pub_state=nh.advertise<communication::state>("communication/state_move",10);
	communication::state msg;

	char rec[MAXSIZE];
	client.open();
	string dev;
	dev=MOVE_ID;
	client.init(UDP_TEST_PORT,UDP_SERVER_IP,rec,dev);
	
	int conn_time=0;
	ros::Rate rate(10);
	
	string t;
	string d;
	while(ros::ok())
  	{
  		if(!client.connect) // 
  		{
  			printf("connect  command\n");
  			t=CONNECT_TYPE;
  			d=CONNECT_DATA;
  			client.sendInfo(	t,d);
			client.wait_back(MAXSIZE,3);

			cout<<endl;

			conn_time++;
			if(conn_time>10)
			{
				msg.break_flag=true;
				pub_state.publish(msg);
				printf("communication failed\n");
			}
  		}
  		else 
  		{	
  			printf("ask command\n");
  			conn_time=0;
			t=STATE_TYPE;
  			d=STATE_DATA;
  			client.sendInfo(t,d);
			client.wait_back(MAXSIZE,TIME_WRONG);  // 6s zhong wu shuju ,ze chonglian
  			
			cout<<endl;

  			msg.break_flag=false;
  			msg.type=STATE_TYPE;
  			msg.data=client.data_in;
  			msg.lenth=client.len+1;//data lenth and type lenth

  			pub_state.publish(msg);
  		}
    		ros::spinOnce();
  		rate.sleep();
  		
  	}
	return 0;
}

bool sendCallback(communication::sendCmd::Request  &req,
         communication::sendCmd::Response &res) 
{
	printf("other command\n");
	printf("AAAAAAAAAAA\n");
            // send command and return somethin 
	client.sendInfo(req.type,req.data);
	client.wait_back(MAXSIZE,TIME_WRONG);

	cout<<endl;

	res.receive=client.receFlag;
	res.state=client.data_in;
 	 return true;
}

