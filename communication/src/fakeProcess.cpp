/*模拟主控
调用plc服务
调用move服务
提供状态信息
*/
#include "ros/ros.h"
#include <communication/sendCmd.h>
#include <communication/state.h>
#include <cstdlib>
using namespace std;
int main(int argc, char **argv)
{
      ros::init(argc, argv, "fake_processor");
      

      ros::NodeHandle n;
      ros::ServiceClient client = n.serviceClient<communication::sendCmd>("send_plc_cmd");
      ros::ServiceClient client1 = n.serviceClient<communication::sendCmd>("send_move_cmd");

      ros::Publisher pub=n.advertise<communication::state>("processor/state",10);

      communication::sendCmd srv;
      communication::state  msg;

      char cmd_type=0x60 ;
      string p="";
      char temp=0x01;
      p+=temp;
     
      
    /*  char cmd_type=0x40 ;
      string p="";
    /*  char cmd_type=0x40 ;
      string p="";
      char temp=0x41; 
      p+=temp;
      temp=0x00;
      p+=temp;
      temp=0x32;
      p+=temp;

      temp=0x42;
      p+=temp;
      temp=0x00;
      p+=temp;
      temp=0x00;
      p+=temp;

      temp=0x43;
      p+=temp;
      temp=0x00;
      p+=temp;
   */
     /* char cmd_type=0x10 ;
      string p="";
      char temp=0x11; 
      p+=temp;
       temp=0x01;
      p+=temp;
       temp=0x12;
      p+=temp;
       temp=0x01;
      p+=temp;
      temp=0x13;
      p+=temp;
       temp=0x01;  p+=temp;  temp=0x02;  p+=temp;*/

     /*char cmd_type=0x30 ;
      string p="";
      char temp=0x01; 
      p+=temp;*/




      srv.request.type+=cmd_type;
      srv.request.data+=p;


     int i=0;
      ros::Rate rate(2);
      if(strcmp(argv[1],"move")==0)
      {     
            printf("move serve\n");
          if (client1.call(srv)&&srv.response.receive)
            {
              ROS_INFO("sucess");
            }
            else    
            {
              ROS_ERROR("Failed ");
              return 1;
            }
    }

    else if(strcmp(argv[1],"pub")==0)
    {

        while (ros::ok())
        {    printf("pub state to shangweiji\n");
                msg.data="";
                msg.data+=0x51;
                msg.data+=0x01;msg.data+=0x01;msg.data+=0x01;msg.data+=0x01;
                msg.data+=0x01;msg.data+=0x01;msg.data+=0x01;msg.data+=0x01;
                msg.data+=0x52; 
                msg.data+=0x01;msg.data+=0x01;
                msg.data+=0x53; 
                msg.data+=0x01;msg.data+=0x01;
                msg.data+=0x54; 
                msg.data+=0x01;msg.data+=0x01;   msg.data+=0x01;msg.data+=0x01;
                msg.data+=0x55; 
                msg.data+=0x01;msg.data+=0x01; 
                msg.data+=0x56; 
                msg.data+=0x01;
                msg.data+=0x57; 
                msg.data+=0x01;
                msg.data+=0x58; 
                msg.data+=0x01;
                msg.data+=0x59; 
                msg.data+=0x01;
                msg.data+=0x5A; 
                msg.data+=0x01;
                msg.data+=0x5B; 
                msg.data+=0x01;
                msg.data+=0x01 ;
                msg.data+=0x5C; 
                msg.data+=0x01;

                pub.publish(msg);
                i++;
                rate.sleep();
            }

    }

    else
    {
        printf("plc serve\n");
          if (client.call(srv)&&srv.response.receive)
            {
              ROS_INFO("plc sucess");
            }
            else
            {
              ROS_ERROR("Failed ");
              return 1;
            }
    }   
    return 0;
}
