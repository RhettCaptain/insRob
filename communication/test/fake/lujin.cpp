#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include <signal.h>
#include <unistd.h>



#define	UDP_TEST_PORT		8001
#define UDP_SERVER_IP 		"192.168.60.169"

using namespace std;

void HexDump(char *buf,int len,int addr) {
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


int main(int argC, char* arg[])
{
	struct sockaddr_in addr;
	int sockfd, len = 0;	
	socklen_t addr_len = sizeof(struct sockaddr_in);		
	char buffer[256];
	char move[256];

	/* 建立socket，注意必须是SOCK_DGRAM */
	if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		perror("socket");
		exit(1);
	}

	/* 填写sockaddr_in*/
	bzero(&addr, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(UDP_TEST_PORT);
	addr.sin_addr.s_addr = inet_addr(UDP_SERVER_IP);
	buffer[0]=0xFA;buffer[1]=0x00;
	buffer[2]=0x21;buffer[3]=0x21;buffer[4]=0x21;buffer[5]=0x21;
             buffer[6]=0x00;buffer[7]=0x02;
	buffer[8]=0x00;buffer[9]=0x01;  //abcd
	buffer[10]=0x0A;
	buffer[11]=0xFF;
	
	move[0]=0xFA;move[1]=0x00;
	move[2]=0x21;move[3]=0x21;move[4]=0x21;move[5]=0x21;
             move[6]=0x00;move[7]=0x1B;
	move[8]=0x40;move[9]=0x21;  
	move[10]=0x23; move[11]=0x01; move[12]=0x01; move[13]=0x01; move[14]=0x01; 
	move[15]=0x01; move[16]=0x01; move[17]=0x01; move[18]=0x01; 
	move[19]=0x24;move[20]=0x01;move[21]=0x01;
	move[22]=0x25;move[23]=0x01;move[24]=0x01;
	move[25]=0x26;move[26]=0x01;move[27]=0x01;
	move[28]=0x27;move[29]=0x01;
	move[30]=0x28;move[31]=0x01;move[32]=0x01;
	move[33]=0x29;move[34]=0x01;
	move[35]=0x0A;move[36]=0xFF;

	int i=0;
	while(1) {

		if(i<5)
		{
			len=12;
			sendto(sockfd, buffer, len, 0, (struct sockaddr *)&addr, addr_len);
			
			char *a;
			a=buffer;
			HexDump(a,len,0); 
		}

	
		else 
		{
			len=37;
			
			sendto(sockfd, move, len, 0, (struct sockaddr *)&addr, addr_len);
			char *a;
			a=move;
			HexDump(a,len,0); 
		}
		sleep(2); /*睡眠2秒*/
		
		i++;

	}

	return 0;
}

// ----------------------------------------------------------------------------
// End of udp_client.c

