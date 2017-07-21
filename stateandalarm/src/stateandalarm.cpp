#include "ros/ros.h"
#include "communication/state.h"
#include "stateandalarm/state.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include "pkg_msgs/MsgOdometrySensor.h"
#include "processor/moveorder.h"   //主控制器提供的消息
#include <sstream>
#include <stdlib.h>
#include <string>
#include <cmath>

using namespace std;

int hightmp=20;  //机器人本体高温线
int lowtmp=-5;  //机器人本体低温线
//float radio=0.15;  //机器人轮半径
float spdfactor=0.0008;//机器人转速转化为线速度的系数
float grid=M_PI*0.3/200000;  //编码器单位栅格长度
bool first=true;

void HexDump(char *buf,int len,int addr)
 {
    int i,j,k;
    char binstr[80];
 
    for (i=0;i<len;i++) {
        if (0==(i%16)) {
            sprintf(binstr,"%08x -",i+addr);
            sprintf(binstr,"%s %02x",binstr,(unsigned char)buf[i]);
        } else if (15==(i%16)) {
            sprintf(binstr,"%s %02x",binstr,(unsigned char)buf[i]);
            sprintf(binstr,"%s  ",binstr);
            for (j=i-15;j<=i;j++) {
         //       sprintf(binstr,"%s%c",binstr,('!'<buf[j]&&buf[j]<='~')?buf[j]:'.');
            }
            printf("%s\n",binstr);
        } else {
            sprintf(binstr,"%s %02x",binstr,(unsigned char)buf[i]);
        }
    }
    if (0!=(i%16)) {
        k=16-(i%16);
        for (j=0;j<k;j++) {
            sprintf(binstr,"%s   ",binstr);
        }
        sprintf(binstr,"%s  ",binstr);
        k=16-k;
        for (j=i-k;j<i;j++) {
       //     sprintf(binstr,"%s%c",binstr,('!'<buf[j]&&buf[j]<='~')?buf[j]:'.');
        }
        printf("%s\n",binstr);
    }
}


class robotState
{
public:
       unsigned char state[51]={0};//发送给上位机的状态
       unsigned char comstate[3]={0};//通讯状态
       unsigned char robotstate[3]={0};//机器人本体状态
       unsigned char wave[3]={0};//超声波信息
       double testdislf=0;
        double testdislb=0;
         double testdisrf=0;
          double testdisrb=0;
       float VLF=0;
       float VLB=0;
       float VRF=0;
       float VRB=0;
       int CDLF=0;
       int CDLB=0;
       int CDRF=0;
       int CDRB=0;
       int LDLF=0;
       int LDLB=0;
       int LDRF=0;
       int LDRB=0;
       float DLF=0;
       float DLB=0;
       float DRF=0;
       float DRB=0;

       ros::Time lasttime;
       ros::Time curtime;
       int upFlow = pow(2,31)-1;
       int downFlow = -pow(2,31);
       
       
       robotState()
       {

        state[0]=0x01;//接收正常
        state[1]=0x51;//机器人坐标，cm
        state[10]=0x52;//机器人位姿
        state[13]=0x53;//机器人速度
        state[16]=0x54;//云台高度
        state[19]=0x55;//机器人电量
        state[21]=0x56; //机内温度
        state[26]=0x57;//急停状态
        state[28]=0x58;//超声波状态
        state[30]=0x59;//机器人控制模式
        state[32]=0x5A;//是否到达目标点
        state[34]=0x5B;//机器人状态
        state[49]=0x5C;//路径片id
        state[50]=0xFF;

     pub_state = n.advertise<communication::state>("processor/state",1000);
     
     pub_sensor = n.advertise<pkg_msgs::MsgOdometrySensor>("topic_odometry_sensor",1000);

     sub1 = n.subscribe("topic_robot_pose",1, &robotState::statecallback1,this);
//订阅机器人位姿信息
     sub2 = n.subscribe("odom",1, &robotState::statecallback2,this);
//订阅机器人实时速度消息
     sub3 = n.subscribe("communication/state_plc", 1, &robotState::statecallback3,this);
//订阅plc板的状态消息
     sub4 = n.subscribe("communication/state_move", 1, &robotState::statecallback4,this);
//订阅运动控制板的状态消息
     sub5 = n.subscribe("state_route",1, &robotState::statecallback5,this);
//订阅路径规划的信息 
   // sub7 = n.subscribe("lift",1, &robotState::statecallback6,this);
//订阅升降机消息
      sub6 = n.subscribe("moveorder",1, &robotState::statecallback6,this);//接收主控板消息
       }
   
void statecallback1(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg1)  //获取机器人坐标以及姿态角度；
{
   
   ROS_INFO("receive msg from nav\n");
   double X,Y,th;

   X = msg1->pose.pose.position.x;
   Y = msg1->pose.pose.position.y;
   th = tf::getYaw(msg1->pose.pose.orientation);//实际使用
   int Xcm=0;//位置数据待处理
   int Ycm=0;
   Xcm=X*100;
   Ycm=Y*100;
   printf("the locationx is %f,   the location Y is %f,  th is %f\n",X,Y,th);
   int thdu=0;//姿态数据待处理
   thdu=10*th*360/(2*M_PI);//确认是否弧度制，目前依弧度处理0120

   state[0]=0x01;//接收正常
   state[1]=0x51;//机器人坐标，cm
   state[2]=Xcm>>24;
   state[3]=Xcm>>16;
   state[4]=Xcm>>8;
   state[5]=Xcm;
   state[6]=Ycm>>24;
   state[7]=Ycm>>16;
   state[8]=Ycm>>8;
   state[9]=Ycm;
   state[10]=0x52;//机器人位姿
   state[11]=thdu>>8;
   state[12]=thdu;
   ROS_INFO(" location received!");
   this->pubstate();
}

void statecallback2(const nav_msgs::Odometry::ConstPtr& msg2) //  //获取机器人速度信息
{
  
  ROS_INFO("receive msg from odom!");
  double V=0;
  double Vx=msg2->twist.twist.linear.x;
  double Vy=msg2->twist.twist.linear.y;
  V=sqrt(Vx*Vx+Vy*Vy);//单位 m/s
  int Vcms=0;
  Vcms=V*100;//单位 cm/s
 
  state[13]=0x53;//机器人速度
  state[14]=Vcms>>8;
  state[15]=Vcms;
  ROS_INFO("vrctor received!"); 
   this->pubstate();
}

void statecallback3(const communication::state::ConstPtr& msg3)//获取PLC 板信息；
{
   
  ROS_INFO("receive msg from PLC!");
  char plc[50];
  memcpy(plc,msg3->data.c_str(),msg3->data.size()+1);
  comstate[0]=0x00;
  if(msg3->break_flag){comstate[0]=0x02;}//与PLC通讯断开
 
  state[19]=0x55;//机器人电量
  state[20]=plc[37]; 

  state[21]=0x56; //机内温度
  state[22]=plc[20]; //电池温度
  state[23]=plc[21]; 
  state[24]=0x00; //工控机仓温度 to be solved!!!!!
  state[25]=0x00;
  
 
  unsigned int bstate[6]={0};
  int Bstate=0;
  bstate[0]=(int)0x0000;
  if(plc[6]=='1'){bstate[1]=(int)0x0080;}
  if(plc[5]=='1'){bstate[2]=(int)0x0100;}
  if(plc[4]=='1'){bstate[3]=(int)0x0200;}
  if(plc[3]=='1'){bstate[4]=(int)0x0400;}
  if(plc[2]=='1'){bstate[5]=(int)0x0080;}
  Bstate=bstate[0]+bstate[1]+bstate[2]+bstate[3]+bstate[4]+bstate[5];
  state[39]=Bstate>>8;//机器人电池错误状态
  state[40]=Bstate;
  
  int robottmp=0;
  int tmp1=0;
  int tmp2=0;
  int tmp3=0;
  int tmp4=0;
  tmp1=((tmp1 | plc[28])<<8)|plc[29];
  tmp2=((tmp2 | plc[30])<<8)|plc[31];
  tmp3=((tmp3 | plc[32])<<8)|plc[33];
  tmp4=((tmp4 | plc[34])<<8)|plc[35];
  robottmp=(tmp1+tmp2+tmp3+tmp4)/4;
  robotstate[0]=0x00;
  robotstate[1]=0x00;
  if(robottmp>=hightmp) {robotstate[0]=0x20;}
  if(robottmp<=lowtmp)  {robotstate[1]=0x40;}
  state[37]=0x00;
  state[38]=robotstate[0] | robotstate[1]; 
 
  ROS_INFO("PLC receivrd!");
   this->pubstate();
}  


void statecallback4(const communication::state::ConstPtr& msg4)
{
//获取运动控制板信息；
   char *buf;
   buf=(char *)msg4->data.c_str();
   int out_l=msg4->data.size();
 //  printf("msg4 is: \n");
  // HexDump(buf,out_l,0);
   
 // ROS_INFO("receive msg from move!");
  unsigned char mov[100];
  
  memcpy(mov,msg4->data.c_str(),msg4->data.size()+1);
 
 //  printf("mov is: \n");
 //  HexDump(mov,out_l,0);
  state[26]=0x57;//急停状态
  state[27]=mov[5];
  state[28]=0x58;//超声波状态
  state[29]=0x00;
  if((mov[2]==0x00))
  {wave[0]=0x00;}
  else 
  {wave[0]=0x04;}//跌落风险
  if((mov[3]&0xF0)==0x00)
  {wave[1]=0x00;}
  else
  {wave[1]=0x01;}//前方障碍
  if((mov[3]&0x0F)==0x00)
  {wave[2]=0x00;}
  else
  {wave[2]=0x02;}//后方障碍
  
  state[29]=(wave[0]|wave[1]|wave[2]);
  
  if(mov)
  state[34]=0x5B;//机器人状态
  
  comstate[1]=0x00;
  if(msg4->break_flag) {comstate[1]=0x04;}//与运控断开
  
  unsigned int mdstate[65]={0};
  unsigned int lumd=0;
  unsigned int ldmd=0;
  unsigned int wumd=0;
  unsigned int wdmd=0;
  int i=0;
  int j=0;
  int k=0;
  int l=0;
  mdstate[0]=0;
  if(mov[50]==0x01){mdstate[1]=(int)0x0001;}// 左上电机
  if(mov[50]==0x02){mdstate[2]=(int)0x0002;}
  if(mov[50]==0x04){mdstate[3]=(int)0x0004;}
  if(mov[50]==0x08){mdstate[4]=(int)0x0008;}
  if(mov[50]==0x10){mdstate[5]=(int)0x0010;}
  if(mov[50]==0x20){mdstate[6]=(int)0x0020;}
  if(mov[50]==0x40){mdstate[7]=(int)0x0040;}
  if(mov[50]==0x80){mdstate[8]=(int)0x0080;}
  if(mov[49]==0x01){mdstate[9]=(int)0x0100;}
  if(mov[49]==0x02){mdstate[10]=(int)0x0200;}
  if(mov[49]==0x04){mdstate[11]=(int)0x0400;}
  if(mov[49]==0x08){mdstate[12]=(int)0x0800;}
  if(mov[49]==0x10){mdstate[13]=(int)0x1000;}
  if(mov[49]==0x20){mdstate[14]=(int)0x2000;}
  if(mov[49]==0x40){mdstate[15]=(int)0x4000;}
  if(mov[49]==0x80){mdstate[16]=(int)0x8000;}
  for(i=1;i<17;i++){lumd+=mdstate[i];}
  state[41]=lumd>>8;
  state[42]=lumd;
  if(mov[54]==0x01){mdstate[17]=(int)0x0001;}// 左下电机
  if(mov[54]==0x02){mdstate[18]=(int)0x0002;}
  if(mov[54]==0x04){mdstate[19]=(int)0x0004;}
  if(mov[54]==0x08){mdstate[20]=(int)0x0008;}
  if(mov[54]==0x10){mdstate[21]=(int)0x0010;}
  if(mov[54]==0x20){mdstate[22]=(int)0x0020;}
  if(mov[54]==0x40){mdstate[23]=(int)0x0040;}
  if(mov[54]==0x80){mdstate[24]=(int)0x0080;}
  if(mov[53]==0x01){mdstate[25]=(int)0x0100;}
  if(mov[53]==0x02){mdstate[26]=(int)0x0200;}
  if(mov[53]==0x04){mdstate[27]=(int)0x0400;}
  if(mov[53]==0x08){mdstate[28]=(int)0x0800;}
  if(mov[53]==0x10){mdstate[29]=(int)0x1000;}
  if(mov[53]==0x20){mdstate[30]=(int)0x2000;}
  if(mov[53]==0x40){mdstate[31]=(int)0x4000;}
  if(mov[53]==0x80){mdstate[32]=(int)0x8000;}
  for(j=17;j<33;j++){ldmd+=mdstate[i];}
  state[43]=ldmd>>8;
  state[44]=ldmd;
  if(mov[58]==0x01){mdstate[33]=(int)0x0001;}// 右上电机
  if(mov[58]==0x02){mdstate[34]=(int)0x0002;}
  if(mov[58]==0x04){mdstate[35]=(int)0x0004;}
  if(mov[58]==0x08){mdstate[36]=(int)0x0008;}
  if(mov[58]==0x10){mdstate[37]=(int)0x0010;}
  if(mov[58]==0x20){mdstate[38]=(int)0x0020;}
  if(mov[58]==0x40){mdstate[39]=(int)0x0040;}
  if(mov[58]==0x80){mdstate[40]=(int)0x0080;}
  if(mov[57]==0x01){mdstate[41]=(int)0x0100;}
  if(mov[57]==0x02){mdstate[42]=(int)0x0200;}
  if(mov[57]==0x04){mdstate[43]=(int)0x0400;}
  if(mov[57]==0x08){mdstate[44]=(int)0x0800;}
  if(mov[57]==0x10){mdstate[45]=(int)0x1000;}
  if(mov[57]==0x20){mdstate[46]=(int)0x2000;}
  if(mov[57]==0x40){mdstate[47]=(int)0x4000;}
  if(mov[57]==0x80){mdstate[48]=(int)0x8000;}
  for(k=33;k<49;k++){wumd+=mdstate[i];}
  state[45]=wumd>>8;
  state[46]=wumd;
  if(mov[62]==0x01){mdstate[49]=(int)0x0001;}// 右下电机
  if(mov[62]==0x02){mdstate[50]=(int)0x0002;}
  if(mov[62]==0x04){mdstate[51]=(int)0x0004;}
  if(mov[62]==0x08){mdstate[52]=(int)0x0008;}
  if(mov[62]==0x10){mdstate[53]=(int)0x0010;}
  if(mov[62]==0x20){mdstate[54]=(int)0x0020;}
  if(mov[62]==0x40){mdstate[55]=(int)0x0040;}
  if(mov[62]==0x80){mdstate[56]=(int)0x0080;}
  if(mov[61]==0x01){mdstate[57]=(int)0x0100;}
  if(mov[61]==0x02){mdstate[58]=(int)0x0200;}
  if(mov[61]==0x04){mdstate[59]=(int)0x0400;}
  if(mov[61]==0x08){mdstate[60]=(int)0x0800;}
  if(mov[61]==0x10){mdstate[61]=(int)0x1000;}
  if(mov[61]==0x20){mdstate[62]=(int)0x2000;}
  if(mov[61]==0x40){mdstate[63]=(int)0x4000;}
  if(mov[61]==0x80){mdstate[64]=(int)0x8000;}
  for(l=49;l<65;l++){wdmd+=mdstate[i];}
  state[47]=wdmd>>8;
  state[48]=wdmd;
  
  this->pubstate();
  
  int WLF=0;
  int WLB=0;
  int WRF=0;
  int WRB=0;
 

  WLF=(short)((0|mov[20])<<8)|mov[21];
  WLB=(short)((0|mov[24])<<8)|mov[25];
  WRF=(short)((0|mov[22])<<8)|mov[23];
  WRB=(short)((0|mov[26])<<8)|mov[27];
//  printf("WLB is %d\n",WLB);
  VLF=spdfactor*WLF;
  VLB=spdfactor*WLB;
  VRF=spdfactor*WRF;
  VRB=spdfactor*WRB;
  
  CDLF=0;CDLB=0;CDRB=0;CDRF=0;
  
  CDLF=(((((((CDLF&0)|mov[64])<<8)|mov[65])<<8)|mov[66])<<8)|mov[67];
  CDLB=(((((((CDLB&0)|mov[72])<<8)|mov[73])<<8)|mov[74])<<8)|mov[75]; //左后编码器
  CDRF=(((((((CDRF&0)|mov[68])<<8)|mov[69])<<8)|mov[70])<<8)|mov[71]; //右前编码器
  CDRB=(((((((CDRB&0)|mov[76])<<8)|mov[77])<<8)|mov[78])<<8)|mov[79]; //右后编码器
  
  
  this->pubsensor();
  
   
}


void statecallback5(const stateandalarm::state::ConstPtr& msg5)
{
  //获取路径规划信息；
  
 
  char rout[3]={0};
  memcpy(rout,msg5->data.c_str(),msg5->data.size()+1);
  state[32]=0x5A;//是否到达目标点
  state[33]=rout[0];
  state[49]=0x5C;
  state[50]=rout[1];
//  ROS_INFO("route received!");
   this->pubstate();
}  

void statecallback6(const processor::moveorder::ConstPtr& msg6)//主控板消息的回调函数
{

   state[30]=0x59;//机器人控制模式
   if(msg6->state == "automatic")
	{
	   state[31]=0x00;
	}
	else 
	{
	   state[31]=0x01;
	}
	this->pubstate();
}
/*
void statecallback6(const stateandalarm::state::ConstPtr& msg6)
{
    char lift[2]={0};
   strcpy(lift,msg6->data.c_str());
    state[16]=0x54;//云台高度
    state[17]=lift[0];
    state[18]=lift[1];
}
*/
void pubstate()
{
   
   state[35]=0x00;
   state[36]=comstate[0]|comstate[1];
   
   
   communication::state robstate;
   robstate.type=0x50;
   robstate.lenth=51;
   robstate.data="";
   int m=0;
   for(m=0;m<51;m++){robstate.data+=state[m];}

   pub_state.publish(robstate);
 
}  

void pubsensor()
{

  if( (LDLF>upFlow/2)&&(CDLF<downFlow/2) )
  {
  	
  	DLF= grid*((CDLF-downFlow)+(upFlow-LDLF)+1);
  }
  else if( (LDLF<downFlow/2)&&(CDLF>upFlow/2))
  {
  	DLF= -grid*((LDLF-downFlow)+(upFlow-CDLF)+1);
  }
  else
  {
  	DLF=grid*(CDLF-LDLF);
  }
  
  
  if( (LDLB>upFlow/2)&&(CDLB<downFlow/2) )
  {
  	DLB= grid*((CDLB-downFlow)+(upFlow-LDLB)+1);
  }
  else if( (LDLB<downFlow/2)&&(CDLB>upFlow/2))
  {
  	DLB= -grid*((LDLB-downFlow)+(upFlow-CDLB)+1);
  }
  else
  {
  	DLB=grid*(CDLB-LDLB);
  }
  
  
  if( (LDRF>upFlow/2)&&(CDRF<downFlow/2) )
  {
  	DRF= grid*((CDRF-downFlow)+(upFlow-LDRF)+1);
  }
  else if( (LDRF<downFlow/2)&&(CDRF>upFlow/2))
  {
  	DRF= -grid*((LDRF-downFlow)+(upFlow-CDRF)+1);
  }
  else
  {
  	DRF=grid*(CDRF-LDRF);
  }
  
  
  if( (LDRB>upFlow/2)&&(CDRB<downFlow/2) )
  {
  	DRB= grid*((CDRB-downFlow)+(upFlow-LDRB)+1);
  }
  else if( (LDRB<downFlow/2)&&(CDRB>upFlow/2))
  {
  	DRB= -grid*((LDRB-downFlow)+(upFlow-CDRB)+1);
  }
  else
  {
  	DRB=grid*(CDRB-LDRB);
  }
  if(first)
  {
    DLF=DLB=DRF=DRB=0;
    first=false;
  }
  
  pkg_msgs::MsgOdometrySensor sensor;
  sensor.type="ODOMETER";
  sensor.vlf=VLF;
  sensor.vlb=VLB;
  sensor.vrf=-VRF;
  sensor.vrb=-VRB;
  sensor.dlf=DLF;
  sensor.dlb=DLB;
  sensor.drf=-DRF;
  sensor.drb=-DRB;
  sensor.vth=0;
  sensor.th=0;
  
  
  testdislf+=DLF;
  testdislb+=DLB;
  testdisrf+=DRF;
  testdisrb+=DRB;
  

  pub_sensor.publish(sensor);

  LDLF=CDLF;
  LDLB=CDLB;
  LDRF=CDRF;
  LDRB=CDRB;

  
}
   
private:
     
     ros::NodeHandle n;
  
     ros::Publisher pub_state;
     ros::Publisher pub_sensor;

     ros::Subscriber sub1;
     ros::Subscriber sub2;
     ros::Subscriber sub3;
     ros::Subscriber sub4;
     ros::Subscriber sub5;
     ros::Subscriber sub6;
};


int main(int argc, char**argv)
{
	ros::init(argc, argv, "stateandalarm");
        
        robotState recOb;
	ros::MultiThreadedSpinner s(8);
        ros::spin(s);
	
	return 0;

}
   
  
     




 



















