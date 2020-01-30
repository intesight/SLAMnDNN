/* File name: server.c*/
//服务器端
#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<errno.h>
#include<unistd.h>
#include<sys/types.h>
#include<sys/socket.h>
#include<netinet/in.h>
#include <sys/time.h>
#include <time.h> 

#define MAXLINE 4096
#define PORT 10004
#define size (6)

 void sysTime(void)
{
    struct timeval tv;
    struct timezone tz;
    struct tm *t;

    gettimeofday(&tv, &tz);
    t = localtime(&tv.tv_sec);
    printf("\ntime_now:%d-%d-%d %d:%d:%d.%ld\n", 1900+t->tm_year, 1+t->tm_mon, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec, tv.tv_usec);
}

 
int server(float sendData[size]){
	//定义服务器监听套接字和连接套接字
	int listen_fd = -1, connect_fd = -1;//初始化为-1
	struct sockaddr_in servaddr;//定义服务器对应的套接字地址
	//服务器接收和发送缓冲区
	//char sendbuf[MAXLINE], recbuf[MAXLINE];
	
	
	//初始化套接字地址结构体
	memset(&servaddr, 0, sizeof(servaddr));
	servaddr.sin_family = AF_INET;//IPv4
	servaddr.sin_port = htons(PORT);//设置监听端口
	servaddr.sin_addr.s_addr = htonl(INADDR_ANY);//表示接收任意IP的连接请求
 
	//创建套接字
	if((listen_fd = socket(AF_INET, SOCK_STREAM, 0)) == -1){
		//如果创建套接字失败，返回错误信息
		//strerror(int errnum)获取错误的描述字符串
		printf("create socket error: %s(error: %d)\n", strerror(errno), errno);
		exit(0);
	}


	int on =1;
	int s = setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
	if(s < 0){
		perror("setsocketopt");
		exit(6);
	}
	
    
	//绑定套接字和本地IP地址和端口
	if(bind(listen_fd, (struct sockaddr*)&servaddr, sizeof(servaddr)) == -1){
		//绑定出现错误
		printf("bind socket error: %s(error: %d)\n", strerror(errno), errno);
		exit(0);
	}
 
	//使得listen_fd变成监听描述符
	if(listen(listen_fd, 10) == -1){
		printf("listen socket error: %s(error: %d)\n", strerror(errno), errno);
		exit(0);
	}
 
	//accept阻塞等待客户端请求
	printf("等待客户端发起连接\n");
 
	while(1){
		if((connect_fd = accept(listen_fd, (struct sockaddr*)NULL, NULL)) == -1){
			printf("accept socket error: %s(error: %d)\n", strerror(errno), errno);
			continue;
		}

 
		//可以一直保持连接
		//while(1){
			//读取客户端发来的信息
			//ssize_t len = read(connect_fd, recbuf, sizeof(recbuf));
			//if(len < 0){
			//	if(errno == EINTR){
			//		continue;
			//	}
			//	exit(0);
			//}
 
			//printf("接收客户端的请求：%s\n", recbuf);
 
			//向客户端发送信息
			
			printf("向客户端发送信息：");
			for(int i = 0; i < size; ++i)
 			{
 				printf("%f\n",sendData[i]);
 			}
			//fgets(sendbuf, sizeof(sendbuf), stdin);
			write(connect_fd, (char *) sendData, size*sizeof(float));
			sysTime();		
		//}
 
		//关闭连接套接字
		close(connect_fd);
		break;
	}
 
	//关闭监听套接字
	close(listen_fd);
}




int main()
{
	float sendData[size]={0,1,3,3,5};
	server(sendData);

}