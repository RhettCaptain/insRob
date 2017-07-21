#include "ros/ros.h"
#include "communication/command.h"
#include "processor/moveorder.h"
//#include "processor/stopflag.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "pkg_srvs/SrvMode.h"
#include "communication/sendCmd.h"
#include <sstream>
#include <string.h>
#include <iostream>

using namespace std;

//指令类型
unsigned char msgtype2=0x10;//启动配置指令
unsigned char msgtype3=0x20;//巡检任务指令
unsigned char msgtype4=0x30;//自由遥控指令
unsigned char msgtype5=0x40;//路径遥控指令
unsigned char msgtype7=0x60;//重启指令
unsigned char msgtype8=0x70;//停车指令
unsigned char msgtype9=0x80;//模式切换指令
unsigned char msgtype10=0x90;//返航指令
unsigned char msgtype11=0xA0;//照明设备指令
//遥控指令分类
unsigned char ordtype1=0x31;//遥控机器人运动指令
unsigned char ordtype2=0x32;//调整机器人速度指令
unsigned char ordtype5=0x35;//设置云台高度指令
unsigned char ordtype8=0x38;//开启构图指令
unsigned char ordtype9=0x39;//开启激光量测指令
unsigned char ordtypeA=0x3A;//开启激光定位指令
//标号0x31消息（机器人遥控移动）内容分类
unsigned char movtype0=0x00;//停止
unsigned char movtype1=0x01;//前进
unsigned char movtype2=0x02;//后退
unsigned char movtype3=0x04;//左转
unsigned char movtype4=0x08;//右转
unsigned char movtype5=0x05;//前左
unsigned char movtype6=0x09;//前又
unsigned char movtype7=0x06;//后左
unsigned char movtype8=0x0A;//后右
//机器人遥控模式下子设定的线速度与角速度
#define Vmax 100 //机器人最大线速度 单位：cm/s
//#define Vup 50 //机器人自设定基准前进线速度 单位：cm/s
#define Wlft 1000 //机器人自设定基准左转角速度 单位：0.01rad/s
//#define Vdw -50 //机器人自设定基准后退线速度 单位：cm/s
#define Wrt -1000 //机器人子设定基准右转角速度 单位：0.01rad/s


class Processor
{

public:
  Processor()
  {
        
      pub = n.advertise<processor::moveorder>("moveorder", 1000); //发布给运动控制模块、升降机模块 
      
      
      
	  //Topic to subscribe
      sub1 = n.subscribe("topic_robot_pose",1, &Processor::callback1,this);
//接受机器人位姿信息 
      sub2 = n.subscribe("communication/cmd", 5, &Processor::callback2, this);
//接收通讯模块信息     
 }

string strtf(int lenth, char *tfdata)  //数组转化为字符串；
{
    string b="";
    int i=0;
    for(i=0;i<lenth;i++)
    {
       b+=tfdata[i];
    }
   
    return b;
}





//void pubmoveorder()
//{
//    processor::moverorder moverorder;
//    moveorder.state+=STATE;
 //   moveorder.pointtype=POINTTYPE;
 //   moveorder.location=LOCATION;
//    moveorder.pose=POSE;
 //   moveorder.speed=SPEED;
//    moveorder.height=HEIGHT;
//    moveorder.pointlevel=POINTLEVEL;
   
//    pub.publish(moveorder);
//    ROS_INFO("moveorder published!\n");

//}


 void callback1(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg1)//获取机器人坐标以及姿态角度；
{
  // ROS_INFO("receive msg from nav\n");
   //保存机器人位置信息；
}

void callback2(const communication::command::ConstPtr& msg)
{ 
       ros::ServiceClient clientp = n.serviceClient<communication::sendCmd>("send_plc_cmd");//给PLC的发送消息的服务
       ros::ServiceClient clientm = n.serviceClient<communication::sendCmd>("send_move_cmd");//给运动控制板发送消息的服务
       ros::ServiceClient clientn = n.serviceClient<pkg_srvs::SrvMode>("srv_mode");//调用导航模块服务
       
	if ( msg->wrong_flag)
	    {
	    ROS_INFO("waiting for communication reconnecting!/n");
	    }	
        if (msg->break_flag)//通讯中断，返回充电屋,待处理
           {
            ROS_INFO("communication fail!/n");
           //调用返回服务
	  //  ros::ServiceClient client = n.serviceClient<processor::SrvReturnRoute>("srv_return_route");//实际需要修改
          //  processor::SrvReturnRoute srvreturn;//待处理！！！！！！！！！！！！！！！！！
	    }
          char data[1000]={0};
          ROS_INFO("%s\n",msg->data.c_str());
          
          memcpy(data,msg->data.c_str(),msg->data.size()+1);
         // ROS_INFO("%s\n",data);
         // ROS_INFO("%s", msg->data.c_str());//测试用
         
       //   unsigned int inttype=0;
       //   inttype=(unsigned int)(msg->type[0]);
       //   ROS_INFO("MSG->TYPE is %d\n",inttype);  
          unsigned char testtype=msg->type[0];
          
          if(testtype==msgtype2)//启动参数配置//
          {
            char setp[11]={0};
            setp[0]=0x11;
            setp[1]=data[1];//电量阈值
            setp[2]=0x12;
            setp[3]=data[3];//电量报警阈值
            setp[4]=0x13;
            setp[5]=data[5];//通信中断时间阈值
            setp[6]=0x14;
            setp[7]=data[7];//通讯故障时间阈值
            setp[8]=0x15;
            setp[9]=data[9];//机体温度
            setp[10]=data[10];//机体温度
            communication::sendCmd srvp;//发送给plc
            srvp.request.type=0x10;
            srvp.request.lenth=11;
            srvp.request.data="";
            srvp.request.data+=strtf(srvp.request.lenth, setp);
            if(clientp.call(srvp)&&srvp.response.receive)
             {
                 ROS_INFO("setup sucess");
              }
            else 
             {
                ROS_INFO("call again");
              }
            char setmov[7];
            setmov[0]=0x11;
            setmov[1]=data[5];//通讯中断阈值
            setmov[2]=0x12;
            setmov[3]=data[7];//通讯故障阈值
            setmov[4]=0x13;
            setmov[5]=data[12];//超声波报警距离
            setmov[6]=data[13];//超声波报警距离
           communication::sendCmd srvm;//发送给运控板
           srvm.request.type=0x10;
           srvm.request.lenth=7;
           srvm.request.data="";           
           srvm.request.data+=strtf(srvm.request.lenth, setmov);
           if(clientm.call(srvm)&&srvm.response.receive)
            {
              ROS_INFO("setup sucess");
             }
           else 
            {
                ROS_INFO("call again");
            }
          }
         else if(testtype==msgtype3||testtype==msgtype5)//巡检任务指令//
          {
             processor::moveorder moveorder; 
             moveorder.routeid=data[0];
             printf("route id is %c", data[0]);      
             char datak[1000]={0};
             
             for(int dn=0;dn<999;dn++)
             {
                 datak[dn]=data[dn+1];                
             }

       //      ROS_INFO("Monitoring JOB1\n");
             int i=0;
             char type[100];
             char *ty=type;
             char location[999];
             char *lo=location;
             char pose[999];
             char *po=pose;
             char speed[100];
             char *sp=speed;
             char height[100];
             char *he=height;
             char level[100];
             char *le=level;
             char *p;
             int k=msg->command_lenth/26;
          
              //  char *buf;
	     // buf=(char *)data_crc.c_str();
	      
           
            for(i=0;i<k;i++)
            {
              p=datak+i*26;
              *ty=*p;ty++;
              for(int j=0;j<8;j++)
               {
                 *lo=*(p+2);
                 lo++;
                 p++;
               }
              *po=*(p+3);po++;p++;
              *po=*(p+3);po++;p++;
              *he=*(p+12);he++;p++;
              *he=*(p+12);he++;p++;
              *sp=*(p+8);sp++;
              *le=*(p+13);le++; 
          

            }
            
           

            moveorder.state="automatic";
            moveorder.pointtype="";
            moveorder.location="";
            moveorder.pose="";
            moveorder.speed="";
            moveorder.height="";
            moveorder.pointlevel="";
         
            moveorder.pointtype+=strtf(k, type);
            moveorder.location+=strtf(8*k, location);
            moveorder.pose+=strtf(2*k,pose);
            moveorder.speed+=strtf(k, speed);
            moveorder.height+=strtf(2*k, height);
            moveorder.pointlevel+=strtf(k, level);
            moveorder.stopflag = false;
            
            pub.publish(moveorder);
            
          }
          else if(testtype==msgtype4)//遥控指令//
          {
                char ctrmov[8];
                ctrmov[0]=0x41;//机器人方向
                ctrmov[3]=0x42;//机器人速度
                ctrmov[6]=0x43;//机器人急停指令

                int vec=0;
                vec = (int)data[3]*10;
                int dvec=0;
                dvec = (-1)*vec;
             
              if(data[0]==ordtype1)//控制机器人移动
             {
                
		      if(data[1]==movtype0)//遥控下停止（00）
		      {
		        ctrmov[1]=0x00;
		        ctrmov[2]=0x00;
		        ctrmov[4]=0x00;
		        ctrmov[5]=0x00;
		        ctrmov[7]=0x00; 
			printf("stop/n");

		      }
		      else if(data[1]==movtype1)//遥控下前进（01）
		      {
		        ctrmov[1]=vec>>8;
		        ctrmov[2]=vec;
		        ctrmov[4]=0x00;
		        ctrmov[5]=0x00;
		        ctrmov[7]=0x00;
			printf("go/n");
		      }
		      else if(data[1]==movtype2)//遥控下后退（02）
		      {
		        ctrmov[1]=dvec>>8;
		        ctrmov[2]=dvec;
		        ctrmov[4]=0x00;
		        ctrmov[5]=0x00;
		        ctrmov[7]=0x00;
			printf("back/n");
		      }
		      else if(data[1]==movtype3)//遥控下左转（04）
		      {
		        ctrmov[1]=0x00;
		        ctrmov[2]=0x00;
		        ctrmov[4]=Wlft>>8;
		        ctrmov[5]=Wlft;
		        ctrmov[7]=0x00;
			printf("left\n");
		      }
		      else if(data[1]==movtype4)//遥控下右转（08）
		      {
		        ctrmov[1]=0x00;
		        ctrmov[2]=0x00;
		        ctrmov[4]=Wrt>>8;
		        ctrmov[5]=Wrt;
		        ctrmov[7]=0x00;
			printf("right\n");
		      }
		      else if(data[1]==movtype5)//遥控下前左（05）
		      {
		        ctrmov[1]=vec>>8;
		        ctrmov[2]=vec;
		        ctrmov[4]=Wlft>>8;
		        ctrmov[5]=Wlft;
		        ctrmov[7]=0x00;
			printf("upleft\n");
		      }
		      else if(data[1]==movtype6)//遥控下前右（09）
		      {
		        ctrmov[1]=vec>>8;
		        ctrmov[2]=vec;
		        ctrmov[4]=Wrt>>8;
		        ctrmov[5]=Wrt;
		        ctrmov[7]=0x00;
			printf("upright\n");
		      }
		      else if(data[1]==movtype7)//遥控下后左（06）
		      {
		        ctrmov[1]=dvec>>8;
		        ctrmov[2]=dvec;
		        ctrmov[4]=Wlft>>8;
		        ctrmov[5]=Wlft;
		        ctrmov[7]=0x00;
			printf("bLeft/n");
		      }
		      else if(data[1]==movtype8)//遥控下后右（0A）
		      {
		        ctrmov[1]=dvec>>8;
		        ctrmov[2]=dvec;
		        ctrmov[4]=Wrt>>8;
		        ctrmov[5]=Wrt;
		        ctrmov[7]=0x00;
			printf("bRight/n");
		      }
		      
		      communication::sendCmd srvm;//发送给运控板
		      srvm.request.type=0x40;
		      srvm.request.lenth=8;
                      srvm.request.data="";
		      srvm.request.data+=strtf(srvm.request.lenth, ctrmov);
		                            
		      if(clientm.call(srvm)&&srvm.response.receive)
		      {
		         ROS_INFO("cmd-sent success");
		      }
		       else 
		      {
		        ROS_INFO("send again");
		      }
	       
             } 
               
             else if(data[0]==ordtype5)//设置云台高度
             {
               //待处理
             }
             else if(data[0]==ordtype8)//开启构图
             {
		printf("构图功能/n");         
		//导航模块
              pkg_srvs::SrvMode srvn;
              if(data[1]==0x01)
              {srvn.request.cmd="MAP_ON";}
              else if(data[1]==0x00)
              {srvn.request.cmd="MAP_OFF";}
             if(clientn.call(srvn)&&srvn.response.state)
              {
               ROS_INFO("call sucess");
              }
             else 
              {
                 ROS_INFO("call again");
              }
            
             }
             else if(data[0]==ordtype9)//开启量测定位
             {
	      printf("开启量测定位/n");                 
	      pkg_srvs::SrvMode srvn;
              if(data[1]==0x01)  
              {srvn.request.cmd="ODOM_ON";}
              else if(data[1]==0x00)
              {srvn.request.cmd="ODOM_OFF";}
             if(clientn.call(srvn)&&srvn.response.state)
              {
               ROS_INFO("call sucess");
              }
             else 
              {
                 ROS_INFO("call again");
              }
              
             }
             else if(data[0]==ordtypeA)//开启激光定位
             {
	      printf("开启激光定位/n"); 
              pkg_srvs::SrvMode srvn;
              if(data[1]==0x01)
              {srvn.request.cmd="LASER_ON";}
              else if(data[1]==0x00)
              {srvn.request.cmd="LASER_OFF";}
              if(clientn.call(srvn)&&srvn.response.state)
              {
               ROS_INFO("call sucess");
              }
              else 
               {
                 ROS_INFO("call again");
               }
             
             }
          }
          else if(testtype==msgtype7)//重启指令//
          {
              ROS_INFO("Restart!\n");

               communication::sendCmd srvm;//发送给运控板
               srvm.request.type=0x30;
               srvm.request.lenth=2;
               srvm.request.data=0x01;	      
	          
               if(clientm.call(srvm)&&srvm.response.receive)
              {
                 ROS_INFO("restar the move board sucess");
              }
               else 
              {
                ROS_INFO("send again");
              }
            
            
          }
          else if(testtype==msgtype8)//停车指令
          {
          
               ROS_INFO("STOP!!!!!!!!\n");
                char ctrmov[8];
                ctrmov[0]=0x41;//运动控制处理逻辑指令
                ctrmov[1]=0x00;
                ctrmov[2]=0x00;
                ctrmov[3]=0x42;
                ctrmov[4]=0x00;
                ctrmov[5]=0x00;
                ctrmov[6]=0x43;
                ctrmov[7]=0x01;
               communication::sendCmd srvm;//发送给运控板
               srvm.request.type=0x40;
               srvm.request.lenth=8;
               srvm.request.data="";
	       srvm.request.data+=strtf(srvm.request.lenth, ctrmov);
	          
               if(clientm.call(srvm)&&srvm.response.receive)
              {
                 ROS_INFO("cmd-sent sucess");
              }
               else 
              {
                ROS_INFO("send again");
              }
              
             
             processor::moveorder move;
             move.stopflag=true; 
             pub.publish(move);
              
          }
          else if(testtype==msgtype9)//模式切换指令
          {

             ROS_INFO("model change in------------------\n");
             
            
             char data3=data[0];
             char data4=data[1];
             string modela="automatic";
             string modelb="controlled";
             char testa;
             testa=0x00;
             processor::moveorder moveorder;
             if(data3==0x00)
             {moveorder.state+=modela;}
             else if(data3==0x01)
             {moveorder.state+=modelb;}
             
             moveorder.pointtype+=testa;
             moveorder.location+=testa;
             moveorder.pose+=testa;
             moveorder.speed+=testa;
             moveorder.height+=testa;
             moveorder.pointlevel+=testa;
             
             moveorder.stopflag = true;
    
             pub.publish(moveorder);
            
           
              char ctrmov[8];
              ctrmov[0]=0x41;//运动控制处理逻辑指令
              ctrmov[1]=0x00;
              ctrmov[2]=0x00;
              ctrmov[3]=0x42;
              ctrmov[4]=0x00;
              ctrmov[5]=0x00;
              ctrmov[6]=0x43;
              ctrmov[7]=0x01;
              communication::sendCmd srvm;//发送给运控板
              srvm.request.type=0x40;
              srvm.request.lenth=8;
              srvm.request.data="";
              srvm.request.data+=strtf(srvm.request.lenth, ctrmov);
		      
              if(clientm.call(srvm)&&srvm.response.receive)
               {
                 ROS_INFO("cmd-sent sucess");
               }
              else 
              {
               ROS_INFO("send again");
             }
             ROS_INFO("model change end!!!!!!!!!!!!!\n");
             
            
            

             
           }
          else if(testtype==msgtype10)//返航指令
          {
             ROS_INFO("GO back home!\n");
              //返航指令
          }
          else if(testtype==msgtype11)//照明设备指令
	  {
	    ROS_INFO("LIGHTING!!!!!!!\n");
            communication::sendCmd srvp;//发送给plc
            char io[3];
            io[0]=0x51;
            io[1]=data[0]; 
            srvp.request.type=0x50;
            srvp.request.lenth=2;
            srvp.request.data="";
            srvp.request.data+=strtf(srvp.request.lenth, io);
            if(clientp.call(srvp)&&srvp.response.receive)
             {
               ROS_INFO("setup sucess");
             }
            else 
             {
                ROS_INFO("call again");
             }
          }
           
}
private:
  ros::NodeHandle n;
  ros::Publisher pub;
 
  
  ros::Subscriber sub1;
  ros::Subscriber sub2;

};


int main(int argc, char**argv)
{
	ros::init(argc, argv, "processor");  
	
	Processor SAPObject;
	ros::MultiThreadedSpinner s(3);//待测试
        ros::spin(s);
		
	return 0;

}





















