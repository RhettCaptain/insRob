/*通信模块: 与上位机通信节点
 节点名: command_to_client
 订阅话题:processor/state
 发布话题:communication/cmd
 服务:无		
 功能:启动节点,进入等待连接状态
 	  连接成功,进入等待命令状态,正常接受则循环,否则进行重连
 	  连接中断,进入等待重连状态,重连成功则进入等待命令状态,否则结束该节点,表明与上位机通信故张.
*/
#include <ros/ros.h>
#include <communication/udp_server.h>
#include <communication/command.h>
#define MAXSIZE 1000
#define PORT 8001
#define TIME_BROKEN 30 //300s
#define TIME_WRONG 2//10s

int main(int argc,char**argv)
{
	ros::init(argc,argv,"command_to_client");
	ros::NodeHandle nh; 
	ROS_INFO("start communication !");
	
	UDP_Server server;
	server.open();	
	char send[MAXSIZE];
	char rec[MAXSIZE];
	server.init(PORT,send,rec,nh,TIME_WRONG,TIME_BROKEN);
	
	
	//while(ros::ok()&&!server.broken)
	while(ros::ok())
	{
		server.wait_connect(MAXSIZE,1000);
		server.wait_command(MAXSIZE); //receive message , timeout break;
		printf("wait  reconnect\n");
		server.wait_reconnect(MAXSIZE);
		
	}	
	printf("communication failed\n");
	server.close_ser();
	return 0;
}
