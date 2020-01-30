 #include <stdio.h>
 #include <sys/socket.h>
 #include <sys/types.h>
 #include <string.h>
 #include <netinet/in.h>
 #include <stdlib.h>
 #include <errno.h>
 #include <unistd.h>
 #include <arpa/inet.h>
 #include <sys/time.h>
 #include <time.h>
#include <pthread.h>
  //ros
 #include <ros/ros.h>


 #define size (6)

char* servAddr = "192.168.43.215";
int sendData[6] = {256,257,30,10,20,3};
int tcpClient(int sendData[6],char *servAddr);

 void sysTime(void)
{
    struct timeval tv;
    struct timezone tz;   
    struct tm *t;
     
    gettimeofday(&tv, &tz);
    t = localtime(&tv.tv_sec);
    printf("\ntime_now:%d-%d-%d %d:%d:%d.%ld\n", 1900+t->tm_year, 1+t->tm_mon, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec, tv.tv_usec);
}

 int tcpServer(short recvData[6])
 {
    int listenfd,connfd;
    struct sockaddr_in sockaddr;

    int n;

    memset(&sockaddr,0,sizeof(sockaddr));

    sockaddr.sin_family = AF_INET;
    sockaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    sockaddr.sin_port = htons(10004);

    listenfd = socket(AF_INET,SOCK_STREAM,0);

    bind(listenfd,(struct sockaddr *) &sockaddr,sizeof(sockaddr)); // IPv4/6与UDP/TCP组合，本地协议地址赋予一个套接字

    listen(listenfd,1024);
// 设定频率30Hz
    // ros::Rate rate(1.0);

    // while (1)
    // {
    //     /* code for loop body */
    //     //tcp通信
    //     tcpClient(sendData,servAddr);
    //     ros::spinOnce();
    //     // rate.sleep();
    // }


#if 1

    printf("Please wait for the client information\n");
    //loop while connect with client
    while(1)
    {
        if((connfd = accept(listenfd,(struct sockaddr*)NULL,NULL))==-1)
        {
            printf("accpet socket error: %s errno :%d\n",strerror(errno),errno);
            continue;
        }


        {

            n = recv(connfd,(short *)recvData,size*sizeof(short),0);

            sysTime();
    
            printf("recv msg from client: \n");
            for(int i = 0; i < size; ++i)
            {
                std::cout << "receive data:" << recvData[i] << std::endl;
             //   printf("recive data: %f\n",recvData[i]);
            }
        }
        close(connfd);
        // break;
    }//loop end
    close(listenfd);

#endif

 }
int main(int argc, char *argv[])
{
	short recvData[size]={0};
	tcpServer(recvData);

    // ros::init(argc, argv, "tcp_sever");
}




/*
*function : tcp communicate
*add date:2019-03-23
*/
 int tcpClient(int sendData[6],char *servAddr)
 {
    int *clientData = sendData;
    char *servInetAddr = servAddr;
    int socketfd;
    struct sockaddr_in sockaddr;
    int n;
 
//  if(argc != 2)
//  {
//      printf("client <ipaddress> \n");
//      exit(0);
//  }
 
    socketfd = socket(AF_INET,SOCK_STREAM,0);    //指定期望的通信协议类型，IPv4,字节流套结字，前面组合的系统默认协议
    memset(&sockaddr,0,sizeof(sockaddr));
    sockaddr.sin_family = AF_INET;
    sockaddr.sin_port = htons(10004);
    inet_pton(AF_INET,servInetAddr,&sockaddr.sin_addr);     //地址转换函数
    printf("send message to server\n");
 
    if((connect(socketfd,(struct sockaddr*)&sockaddr,sizeof(sockaddr))) < 0 )//connect函数来建立与指定socket的连接servInetAddr
    {
        printf("connect error %s errno: %d\n",strerror(errno),errno);
        printf("connect error");
        exit(0);
    }
    
    printf("connect success \n");

    if((send(socketfd,(int *)clientData,6*sizeof(int),0)) < 0)
    {
        printf("I failed to send the message\n");
        exit(0);
    }
    sysTime();
    printf("Have sent:\n");
    for(int i = 0; i <6; ++i)
    {	
        printf(" %d\n",clientData[i]);
    }
    close(socketfd);//关闭socket，终止TCP连接

    // printf("exit\n");
    //exit(0);
 }