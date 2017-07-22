#include <ros/ros.h>
#include <sstream>
#include <cmath>
#include "tf/transform_broadcaster.h"
#include "communication/sendCmd.h"
#include "processor/moveorder.h"  //主控制器提供的消息
#include "stateandalarm/state.h"   //发送给状态模块的消息
#include "geometry_msgs/PoseWithCovarianceStamped.h"  //导航模块的位姿消息
#include "geometry_msgs/PoseWithCovariance.h"  //服务里的数据类型
#include "pkg_srvs/SrvGetLine.h"  //两点求直线服务
#include "pkg_srvs/SrvGetYawBias.h" //求偏行角服务
#include <string.h>
#include <iostream>

using namespace std;

#define V 300.0    //定义机器人运行速度 mm/s
#define W 700.0    //定义机器人运行角速度 0.001rad/s
#define T 0.02   //定义每次发送速度指令的时间周期
#define movecount 1   
#define a 0.5
#define b 0.5 
#define Kx 1
#define Ky 15
#define K 2
#define STOPTH 0.02 //旋转允许误差角度（开始停止角度）  单位 rad
#define SLOWTH 0.3   
#define STOPD 0.05  //停车距离    单位 m
#define SLOWD1 1.0  //第一次减速距离   单位 m
//  #define SLOWD2 1.0  //第二次减100速距离   单位 m


class MoveControl
{
public:
        string arvstate; //说明：此数据发送给状态备份模块,为是否到达目标点;
        unsigned char locationX[4]={0x00,0x00,0x00,0x00};
        unsigned char locationY[4]={0,0,0,0};
        unsigned char pose[2]={0,0};
	double x0;//表示机器人实时的x坐标
	double y0;//表示机器人实时y坐标
	double th0;//表示机器人实时姿态角
	
	
	
	bool stopFlag=false;
        
	MoveControl()
	{
	//Topic to publish
        pub_state = n.advertise<stateandalarm::state>("state_route", 1000);//发送给状态模块		

	//Topic to sub
	sub1 = n.subscribe("moveorder",5,&MoveControl::moveordercallback,this);//接收主控板消息
	sub2 = n.subscribe("topic_robot_pose",2,&MoveControl::posecallback,this);//接收导航模块实时位姿
	//sub3 = n.subscribe("stopflag",1,&MoveControl::stopcallback,this);//接收导航模块实时位姿
 
        //service to be used
	client1 = n.serviceClient<pkg_srvs::SrvGetLine>("srv_get_line");//求直线及角度
	clientm = n.serviceClient<communication::sendCmd>("send_move_cmd"); //给运动控制板发送消息的服务
	}



void pubstate()
{
             
             stateandalarm::state statepub;
             statepub.type=0x50;
             statepub.lenth=1;
             statepub.data="";
             statepub.data+=arvstate;
                         
             pub_state.publish(statepub);//发布信息给状态模块
            
}



string strtf(int lenth, char *aa)//用于将字符数组的数据赋给string类型变量 
{
        string bb="";
	int i=0;
	for(i=0;i<lenth;i++)
	{
	  bb+=aa[i];
	}
        return bb;
}



void moveordercallback(const processor::moveorder::ConstPtr& msg)//主控板消息的回调函数
	{
        	
            cout<<"new task"<<endl;
            
            char data[1000]={0};
            memcpy(data,msg->location.c_str(),msg->location.size()+1);
            char strpose[1000]={0};
            memcpy(strpose,msg->pose.c_str(),msg->pose.size()+1);

           stopFlag = msg->stopflag;
           
           int stop=stopFlag;
           
    
           locationX[0]=0;//读出对应点的x坐标
           locationX[1]=0;
           locationX[2]=0;
           locationX[3]=0;
           locationY[0]=0;//读出对应点的y坐标
           locationY[1]=0;
           locationY[2]=0;
           locationY[3]=0;
           
           pose[0]=0;
           pose[1]=0;
           
            
	                  

	 if(msg->state == "automatic")//自动状态
	 {
             arvstate="";
             arvstate+=0x01;
             arvstate+=msg->routeid;
             pubstate();
             printf("state:auto\n");
             
             int pn=0;
             pn=strlen(msg->pointtype.c_str());
             
         
	     int i = 0; 
	     do
	   {

           ros::spinOnce(); 
           if(stopFlag){		   
		  break;           
		  }
		  
           locationX[0]=data[8*i+0];//读出对应点的x坐标
	   locationX[1]=data[8*i+1];
           locationX[2]=data[8*i+2];
           locationX[3]=data[8*i+3];
           locationY[0]=data[8*i+4];//读出对应点的y坐标
           locationY[1]=data[8*i+5];
           locationY[2]=data[8*i+6];
           locationY[3]=data[8*i+7];
           
           pose[0]=strpose[2*i+0];
           pose[1]=strpose[2*i+1];
                
           MoveControl::tarjectoryTracking(locationX,locationY);//调用轨迹跟踪	   
           i++ ;   
	   }while(i<pn);
	   
	   if(!stopFlag)
	   {
	   		  
          	arvstate="";
          	arvstate+=0x02;
          	arvstate+=msg->routeid;
           	pubstate();
           	cout<<"arrived destination"<<endl;
           
          	int dpose=0;
         	dpose=(short)(((dpose|pose[0])<<8)|pose[1]);
           	double ddpose=dpose*M_PI/1800;
           	
           	spinfun(ddpose);
           	
 	  }
	}
		
}
    


void posecallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg1)//位姿消息的回调函数,返回实时位姿//需要设定订阅频率
	{     
	x0 = msg1->pose.pose.position.x ;//机器人实时x坐标
	y0 = msg1->pose.pose.position.y ;//机器人实时y坐标
	th0 = tf::getYaw(msg1->pose.pose.orientation);//机器人实时姿态角
	
	}
	

double getYawBias(double aimtheta) // 求去偏航角 单位 rad 从机器人角度指向目标角度 逆时针方向为正
{
   double yawbias=0;
   double yawb=0;
   yawb=th0-aimtheta;
   if(((-2*M_PI)<=yawb)&&(yawb<(-1*M_PI))){yawbias=-2*M_PI-yawb;}
   else if(((-1*M_PI)<=yawb)&&(yawb<0)){yawbias=-1*yawb;}
   else if((0<=yawb)&&(yawb<M_PI)){yawbias=-1*yawb;}
   else if((M_PI<=yawb)&&(yawb<=2*M_PI)){yawbias=2*M_PI-yawb;}
   
  //printf("current :%f,target :%f ,thetatoturn :%f\n",th0,aimtheta,yawbias);
   return yawbias;
   
}


double getTHe(double targettheta,double realth)
{
   double TH1=0.0;
   double TH2=0.0;
   TH1=targettheta-realth;
   if(((-2*M_PI<=TH1))&&(TH1<(-M_PI)))  {TH2=TH1+2*M_PI;}
   else if((-M_PI<=TH1)&&(TH1<=M_PI))   {TH2=TH1;}
   else if((M_PI<TH1)&&(TH1<=(2*M_PI))) {TH2=-2*M_PI+TH1;}
   
   return TH2;
}


void spinfun(double targetth) //控制机器人旋转
{
        communication::sendCmd automov;
        double thetatoturn=0;
        thetatoturn=getYawBias(targetth);//偏航角
       
       printf("use spinfun\n");
      
        while(thetatoturn>STOPTH||thetatoturn<(-STOPTH))   //先旋转到正确角度
        {
          ros::spinOnce();
           if(stopFlag){
		  char ctrmov2[8]={0};//转动停止
		  ctrmov2[0]=0x41;//运动控制处理逻辑指令
		  ctrmov2[1]=0x00;
		  ctrmov2[2]=0x00;
		  ctrmov2[3]=0x42;
		  ctrmov2[4]=0x00;
		  ctrmov2[5]=0x00;
		  ctrmov2[6]=0x43;
		  ctrmov2[7]=0x00;
		
		  automov.request.type=0x40;
		  automov.request.lenth=8;
		  automov.request.data="";
		  automov.request.data+=strtf(automov.request.lenth,ctrmov2);
		  if(clientm.call(automov)&&automov.response.receive)
		     {
		         ROS_INFO("停止!\n");
		      }
		    else 
		     {
		        ROS_INFO("call again");
		      }
		   break;
           }
          int wfirst=0;
          double spinslowrate=1.0;
          if(((thetatoturn>STOPTH)&&(thetatoturn<SLOWTH))||((thetatoturn<(-STOPTH))&&(thetatoturn>(-SLOWTH))))
          {
             spinslowrate=(0.9/0.28)*fabs(thetatoturn)+0.036;
             //printf("slow rate%f\n",spinslowrate);
          }
          
          
          //判断旋转翻方向
          if(thetatoturn>0){wfirst=W*spinslowrate;}
          if(thetatoturn<0){wfirst=(-1)*W*spinslowrate;}
          
          char ctrmov1[8]={0};
          ctrmov1[0]=0x41;//运动控制处理逻辑指令
          ctrmov1[1]=0x00;
          ctrmov1[2]=0x00;
          ctrmov1[3]=0x42;         
          ctrmov1[4]=wfirst>>8;
          ctrmov1[5]=wfirst;
          ctrmov1[6]=0x43;
          ctrmov1[7]=0x00;
          
          automov.request.type=0x40;
          automov.request.lenth=8;
          automov.request.data="";
          automov.request.data+=strtf(automov.request.lenth,ctrmov1);
          if(clientm.call(automov)&&automov.response.receive)
             {
               //  ROS_INFO("spin cmd sent  sucess");
              }
            else 
             {
                 ROS_INFO("call again");
              }
             thetatoturn=getYawBias(targetth);//偏航角
             
        }
       
      
        
		  char ctrmov2[8]={0};//转动停止
		  ctrmov2[0]=0x41;//运动控制处理逻辑指令
		  ctrmov2[1]=0x00;
		  ctrmov2[2]=0x00;
		  ctrmov2[3]=0x42;
		  ctrmov2[4]=0x00;
		  ctrmov2[5]=0x00;
		  ctrmov2[6]=0x43;
		  ctrmov2[7]=0x00;
		
		  automov.request.type=0x40;
		  automov.request.lenth=8;
		  automov.request.data="";
		  automov.request.data+=strtf(automov.request.lenth,ctrmov2);
		  if(clientm.call(automov)&&automov.response.receive)
		     {
		         //ROS_INFO("停止转动\n");
		      }
		    else 
		     {
		        ROS_INFO("call again");
		      }
		  
          
 }

float footPoint(double x,double y,float _a,float _b,float _c)
{
    float x_out=0.0;
    x_out=(-_a*_b*y+_b*_b*x-_a*_c)/(_a*_a+_b*_b); 
  
    return x_out;
}




void tarjectoryTracking(unsigned char *pointX,unsigned char *pointY)//轨迹跟踪函数//参数为要到达的点的x,y,以及速度
	{
	communication::sendCmd automov;
   	printf("follow the line!\n");
        int ddpointX = 0;
        int ddpointY = 0;//目标点
        double dtheta=0;//目标姿态角
        int startpointx=0;
        int startpointy=0;
        startpointx = x0;
        startpointy = y0;//存储初始点坐标
        //将目标点坐标转换成double型 单位m
	ddpointX =(((((((ddpointX | pointX[0]) << 8) | pointX[1]) << 8 ) |pointX[2]) << 8) | pointX[3]); 
	ddpointY =(((((((ddpointY | pointY[0]) << 8) | pointY[1]) << 8 ) |pointY[2]) << 8) | pointY[3]);

      
       printf("target goal(%d,%d) \n",ddpointX,ddpointY);  
	  
	
	
	float dpointX = ddpointX/100.0;
	float dpointY = ddpointY/100.0;	
	
        pkg_srvs::SrvGetLine getLineService;//两点求直线服务
	geometry_msgs::PoseWithCovariance poseA,poseB;
	poseA.pose.position.x = x0;
	poseA.pose.position.y = y0;
        poseB.pose.position.x = dpointX;
	poseB.pose.position.y = dpointY;
	getLineService.request.poseA = poseA;
	getLineService.request.poseB = poseB;
	if(client1.call(getLineService))
	{
          dtheta = getLineService.response.line[3];  //求得直线角度
         
        }
       
         spinfun(dtheta);
         ros::Duration(1).sleep();
         
           
//此时机器人已经旋转到正确的位姿，下面机器人开始走直线到达目标点

	double distance = sqrt((x0-dpointX)*(x0-dpointX)+(y0-dpointY)*(y0-dpointY));//求到目标点的距离
	double vslowrate=1.0;
	int cn=0;
	
	double min_dis=9999999.0;
	
	bool flag=true;
        while((distance>STOPD)&&flag)
       { 
		ros::spinOnce();
		if(stopFlag){		
		break;
		}
       // cn++;
        double temp_dtheta=0.0;
        poseA.pose.position.x = x0;
	poseA.pose.position.y = y0;
        poseB.pose.position.x = dpointX;
	poseB.pose.position.y = dpointY;
	getLineService.request.poseA = poseA;
	getLineService.request.poseB = poseB;
	
	float lineA=0.0;
	float lineB=0.0;
	float lineC=0.0;
	
	double x_0=0.0;
	double y_0=0.0;
	
	if(client1.call(getLineService))
	{
          temp_dtheta = getLineService.response.line[3];  //求得直线角度
          lineA = getLineService.response.line[0]; 
          lineB = getLineService.response.line[1]; 
          lineC = getLineService.response.line[2]; 
        }
   
      //  x_0=footPoint(x0,y0,lineA,lineB,lineC);
      //  y_0=-1*lineA/lineB*x_0 - lineC/lineB;
        
        
        
        
        double xd = x0 +  V*T*cos(temp_dtheta)*vslowrate/1000.0;
        double yd = y0 +  V*T*sin(temp_dtheta)*vslowrate/1000.0;
        
       
             
	double dspeed = 0.0;
	dspeed = V/1000.0;
        double wd = 0;
        double Xe=0;
        double Ye=0;
        double THe=0;
        double vtd=0;
        double wtd=0;
        
        Xe=(xd-x0)*cos(th0)+(yd-y0)*sin(th0);
        Ye=(-1)*(xd-x0)*sin(th0)+(yd-y0)*cos(th0);
        THe=getTHe(temp_dtheta,th0);
        
    //    printf("temp_dtheta: %f,(y0,yd):(%f,%f) ",temp_dtheta,y0*100,yd*100);
     //   printf("xe:%f,ye%f,the%f\n",Xe,Ye,THe);
         
        wtd =wd+dspeed*(Ky*a*(Ye + K*THe) + (b/K)*sin(THe));
	vtd =dspeed*cos(THe)+Kx*Xe-K*THe*wtd;
	//if((STOPD<distance)&&(distance<SLOWD1)){vslowrate=(0.5/0.95)*distance+0.474;}
        
	int vt=vtd*1000;
	int wt=wtd*1000;
	

       if((STOPD<distance)&&(distance<SLOWD1)){vslowrate=(0.5/0.95)*distance+0.474;}
        int vvt=0;       
        vvt=vt*vslowrate;   

      if(vvt>(2*V)){vvt=2*V;}  //速度限制
        
        char ctrmov3[8]={0};
        ctrmov3[0]=0x41;//运动控制处理逻辑指令
        ctrmov3[1]=vvt>>8;
        ctrmov3[2]=vvt;
        ctrmov3[3]=0x42;
        ctrmov3[4]=wt>>8;
        ctrmov3[5]=wt;
        ctrmov3[6]=0x43;
        ctrmov3[7]=0x00;
        
        automov.request.type=0x40;
        automov.request.lenth=8;
        automov.request.data="";
        automov.request.data+=strtf(automov.request.lenth,ctrmov3);
        
      
        if(clientm.call(automov)&&automov.response.receive)
             {                                                      
                // ROS_INFO("cmd sent sucess");
              //  printf("Distance is %f\n",distance);
              }
            else 
             {
                ROS_INFO("call again");
             }

         distance = sqrt((x0-dpointX)*(x0-dpointX)+(y0-dpointY)*(y0-dpointY));
         
         if(distance<0.5)
         { if(distance<=min_dis)
          {
         	min_dis=distance; 
          } 
         
          else 
          { 
         	if(distance-min_dis> (STOPD/2))
         	{
         		flag=false;
         		printf("flag is flase\n");
         	}
          }
         }
        
    }
        
	
	
	
	//到达目标点停车
        char ctrmov4[8]={0};
        ctrmov4[0]=0x41;//运动控制处理逻辑指令
        ctrmov4[1]=0x00;
        ctrmov4[2]=0x00;
        ctrmov4[3]=0x42;
        ctrmov4[4]=0x00;
        ctrmov4[5]=0x00;
        ctrmov4[6]=0x43;
        ctrmov4[7]=0x00;
       
        automov.request.type=0x40;
        automov.request.lenth=8;
        automov.request.data="";
        automov.request.data+=strtf(automov.request.lenth,ctrmov4);
          if(clientm.call(automov)&&automov.response.receive)
             {
                 //ROS_INFO("smd sent sucess");
               
              }
            else 
             {
                ROS_INFO("call again");
             }
      
	
	}
	
private:
	ros::NodeHandle n;
        ros::Publisher pub_state;
	ros::Subscriber sub1;
	ros::Subscriber sub2;
	
	ros::ServiceClient client1;

	ros::ServiceClient clientm;
};

int main(int argc, char**argv)
{
	ros::init(argc, argv, "move");  
	
	MoveControl SAObject;	
	ros::MultiThreadedSpinner s(5);
        ros::spin(s);
	
	
	return 0;

}

