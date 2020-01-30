 #include <stdio.h>
 #include <sys/socket.h>
 #include <sys/types.h>
 #include <stdlib.h>
 #include <netinet/in.h>
 #include <errno.h>
 #include <string.h>
 #include <arpa/inet.h>
 #include <unistd.h>
 #include <sys/time.h>
 #include <time.h>
 //ros
 #include <ros/ros.h>

//global setting
int sendData[6] = {256,257,30,10,20,3};
char* servAddr = "192.168.43.143";

void sysTime(void);
int tcpClient(int sendData[6],char *servAddr);

int main(int argc, char *argv[])
{
    /* code for main function */
    ros::init(argc, argv, "tcp_comm");
    ros::NodeHandle nh("~");

    // 设定频率1Hz
    ros::Rate rate(0.01);
    while (ros::ok())
    {
        /* code for loop body */
        //tcp通信
        tcpClient(sendData,servAddr);
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}

/*
*function : tcp communicate
*add date:2019-03-23
*/
void sysTime(void)
{
    struct timeval tv;
    struct timezone tz;  
    struct tm *t;
    
    gettimeofday(&tv, &tz);
    t = localtime(&tv.tv_sec);
    printf("\ntime_now:%d-%d-%d %d:%d:%d.%ld\n", 1900+t->tm_year, 1+t->tm_mon, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec, tv.tv_usec);
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
    sockaddr.sin_port = htons(61000);
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