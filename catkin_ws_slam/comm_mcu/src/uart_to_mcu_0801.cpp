#include "stdio.h"
#include "string.h"

#include "fcntl.h"
#include "errno.h"
#include "sys/types.h"
#include "sys/stat.h"
#include "stdlib.h"
#include "stdarg.h"
#include "termios.h"

#include "uart_to_mcu.h"



	 
#define McuOk  1
#define McuErr 2



pthread_mutex_t mcu_data_mutex; 

PCREADDATA 		McuSend_PcReadData,McuSend_PcReadData_Pc;
PCWRITEDATA 	McuReceive_PcWriteData;
int direction;  //0x01 --> left   0x02 --> right

char parking_mode_ok;
int parking_mode_ok_flag;

char printf_flag = 0;
char printf_flag_ext = 0;
char Mcu_receive_FLAG = 1;
char Pc_Tx_Mcu_Nums   =0;



int gear_show_flag;
int fd_uart;
unsigned char connect_success_flag;
unsigned char txBuffer[180];//10//= {0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xaa};//0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99
unsigned char rxBuffer[60];//30
int count;
int heart_beat_rev_flag;
void frame_get_data(unsigned char *r_buffer, unsigned char len);

//below is for mcu --->pc  :20180622------PC---communication
//up is for mcu--->PC

//below is for PC---->MCU
//up is for pc------>mcu
unsigned char Add_Verify(unsigned char *p,unsigned char length);

PCREADDATA 	* get_data_from_mcu(void)
{
//	McuSend_PcReadData.Pc_Read_Flag = 1;
//	usleep(40000);//20ms
	pthread_mutex_lock(&mcu_data_mutex);
	McuSend_PcReadData_Pc = McuSend_PcReadData;
	pthread_mutex_unlock(&mcu_data_mutex);
//	McuSend_PcReadData.Pc_Read_Flag = 0;
	return &McuSend_PcReadData_Pc;
}

void write_data_into_mcu(PCWRITEDATA *p)
{
	McuReceive_PcWriteData = *p;
	McuReceive_PcWriteData.Pc_Write_Flag = 1;
//	write_data();
	usleep(5000);//40000//600000//200000//100000
//	sleep(1);
}

int uart_thread_create(void)
{
	int ret;
	pthread_t id7,id8;
#if 1	
	pthread_attr_t thread_attr;
	struct sched_param schedule_param;

	pthread_attr_init(&thread_attr);
	schedule_param.sched_priority = 99;
	pthread_attr_setinheritsched(&thread_attr, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setschedpolicy(&thread_attr, SCHED_RR);
	pthread_attr_setschedparam(&thread_attr, &schedule_param); 
#endif

	ret = pthread_create(&id7, NULL, Uart_meg_thread,NULL);
	if(ret)
	{		
		printf("Create Uart_meg_thread error!\n");
		return -1;
	}
#if 1
	ret = pthread_create(&id8, NULL, Uart_TX_thread,NULL);
	if(ret)	
	{		
		printf("Create Uart_TX_thread error!\n");		
		return -1;	
	}
#endif
	return 0;
}
void *Uart_meg_thread(void *t) 
{
	int i;
	int rd_count = 0;
	//fd_set fds;
	//struct timeval tv;
	//tv.tv_sec = 0;
 	// tv.tv_usec = 0;
    struct termios tty_attributes;
	

    if (
		((fd_uart = open("/dev/ttyUSB0",O_RDWR|O_NOCTTY&~O_NONBLOCK))>=0)
		||((fd_uart = open("/dev/ttyUSB1",O_RDWR|O_NOCTTY&~O_NONBLOCK))>=0)
		||((fd_uart = open("/dev/ttyUSB2",O_RDWR|O_NOCTTY&~O_NONBLOCK))>=0)
	   )	
	{
        tcgetattr(fd_uart, &tty_attributes);
 
        // c_cflag
        // Enable receiver
        tty_attributes.c_cflag |= CREAD;        
 
        // 8 data bit
        tty_attributes.c_cflag |= CS8;          
 
        // c_iflag
        // Ignore framing errors and parity errors. 
        tty_attributes.c_iflag |= IGNPAR;  
		tty_attributes.c_iflag &=~(ICRNL|IXON);
 
        // c_lflag
        // DISABLE canonical mode.
        // Disables the special characters EOF, EOL, EOL2, 
        // ERASE, KILL, LNEXT, REPRINT, STATUS, and WERASE, and buffers by lines.
 
        // DISABLE this: Echo input characters.
        tty_attributes.c_lflag &= ~(ICANON);     
 
        tty_attributes.c_lflag &= ~(ECHO);      
 
        // DISABLE this: If ICANON is also set, the ERASE character erases the preceding input  
        // character, and WERASE erases the preceding word.
        tty_attributes.c_lflag &= ~(ECHOE);     
 
        // DISABLE this: When any of the characters INTR, QUIT, SUSP, or DSUSP are received, generate the corresponding signal. 
        tty_attributes.c_lflag &= ~(ISIG);      
 
        // Minimum number of characters for non-canonical read.
        tty_attributes.c_cc[VMIN]= 32;//29;//16;            
 
        // Timeout in deciseconds for non-canonical read.
        tty_attributes.c_cc[VTIME]=0;           
 
        // Set the baud rate
        cfsetospeed(&tty_attributes,B115200);     
        cfsetispeed(&tty_attributes,B115200);
		
//		cfsetospeed(&tty_attributes,B19200);     
//        cfsetispeed(&tty_attributes,B19200);
 
        tcsetattr(fd_uart, TCSANOW, &tty_attributes);
		McuSend_PcReadData.Pc_Read_Flag = 0;//for test to be del 
	
		while(1)
		{		
			rd_count = read(fd_uart, rxBuffer, sizeof(rxBuffer));
					
			if(printf_flag)
			printf("rd_count = %d\n", rd_count);

          	if ((rd_count > 0)&&(printf_flag))
		//	if ((rd_count > 0))//&&(printf_flag))
			{
				for (i=0; i<rd_count; i++)
				{
					printf("%x ", rxBuffer[i]);
				}
				printf("\n");

			}
			pthread_mutex_lock(&mcu_data_mutex);
			frame_get_data(rxBuffer, rd_count);
			pthread_mutex_unlock(&mcu_data_mutex);					
		}
    } 
	else 
	{
        fprintf (stderr,"Open error on %s\n", strerror(errno));
        //exit(EXIT_FAILURE);
    } 	
 
    close(fd_uart);  

}

void frame_get_data(unsigned char *r_buffer, unsigned char len)
{
	int i,j=0;
	unsigned char tmpi = 0;//added 
	mcu_soc_frame_t	mcu_frame;
	mcu_soc_frame_t	soc_frame;
	unsigned char 	t_buffer;
	
	short int tmp = 0;//added by che 
	short int tmp2 = 0;//added by che 
	
	mcu_frame.f_start = r_buffer[0];
	mcu_frame.id_flag = r_buffer[1];
    mcu_frame.d_len   = r_buffer[2];
	mcu_frame.data	  = &r_buffer[3];
	//mcu_frame.xor_flag= r_buffer[len-2];

	if((mcu_frame.f_start == F_START_MCU)&&(r_buffer[3+mcu_frame.d_len]==Add_Verify(&r_buffer[3],mcu_frame.d_len)))
	{
		if(printf_flag_ext)
		{
			printf("id_flag = %x \n", mcu_frame.id_flag);
		
			for (i=0; i<mcu_frame.d_len; i++)
			{
				printf("%x ", mcu_frame.data[i]);
			}
			printf("\n");	
		}

		switch (mcu_frame.id_flag)
		{
			case 0x53:
//			if(McuSend_PcReadData.Pc_Read_Flag == 0)
			{
				j=3;
				tmp	 	 = 0;
				tmp 	|=  r_buffer[3];
				tmp      =  tmp<<8;
				tmp 	|=  r_buffer[4];				
				McuSend_PcReadData.parking_rect_point0_x	= tmp;
				
				
				tmp	 	 = 0;
				tmp 	|=  r_buffer[5];
				tmp      =  tmp<<8;
				tmp 	|=  r_buffer[6];				
				McuSend_PcReadData.parking_rect_point0_y	= tmp;
				
				
				tmp	 	 = 0;
				tmp 	|=  r_buffer[7];
				tmp      =  tmp<<8;
				tmp 	|=  r_buffer[8];				
				McuSend_PcReadData.parking_rect_point1_x	= tmp;
				
				
				tmp	 	 = 0;
				tmp 	|=  r_buffer[9];
				tmp      =  tmp<<8;
				tmp 	|=  r_buffer[10];				
				McuSend_PcReadData.parking_rect_point1_y	= tmp;
				
				
				
				tmp	 	 = 0;
				tmp 	|=  r_buffer[11];
				tmp      =  tmp<<8;
				tmp 	|=  r_buffer[12];				
				McuSend_PcReadData.parking_rect_point2_x	= tmp;
				
				
				
				tmp	 	 = 0;
				tmp 	|=  r_buffer[13];
				tmp      =  tmp<<8;
				tmp 	|=  r_buffer[14];				
				McuSend_PcReadData.parking_rect_point2_y	= tmp;	
				
				tmp	 	 = 0;
				tmp 	|=  r_buffer[15];
				tmp      =  tmp<<8;
				tmp 	|=  r_buffer[16];				
				McuSend_PcReadData.parking_rect_point3_x	= tmp;
				
				
				tmp	 	 = 0;
				tmp 	|=  r_buffer[17];
				tmp      =  tmp<<8;
				tmp 	|=  r_buffer[18];				
				McuSend_PcReadData.parking_rect_point3_y	= tmp;											
					
				tmp	 	 = 0;
				tmp 	|=  r_buffer[19];
				tmp      =  tmp<<8;
				tmp 	|=  r_buffer[20];				
				McuSend_PcReadData.TimeStampexTmp[0] = tmp;						

				tmp	 	 = 0;
				tmp 	|=  r_buffer[21];
				tmp      =  tmp<<8;
				tmp 	|=  r_buffer[22];				
				McuSend_PcReadData.TimeStampexTmp[1] = tmp;	
					
				tmp	 	 = 0;
				tmp 	|=  r_buffer[23];
				tmp      =  tmp<<8;
				tmp 	|=  r_buffer[24];				
				McuSend_PcReadData.TimeStampexTmp[2] = tmp;		
				
				McuSend_PcReadData.gear_status_actual 		= r_buffer[25];
				McuSend_PcReadData.gear_status_pre 	  		= r_buffer[26];				
			}	
				
				if(printf_flag)
				{
				printf("rader1_alarm_level_1:x=%hx\n",McuSend_PcReadData.rader1_alarm_level);
				printf("rader1_alarm_level_16:x=%hx\n",McuSend_PcReadData.rader16_alarm_level);
				}
				
				
				printf_flag 	=  r_buffer[27];
				printf_flag_ext =  r_buffer[27];
					
				if(r_buffer[28]==1)
				{
					Mcu_receive_FLAG = McuOk;
					Pc_Tx_Mcu_Nums = 0;
						
					if(printf_flag_ext)
						printf("Mcu_receive_FLAG_53:x=%hx\n",Mcu_receive_FLAG);
				}
				
               	tmp	 	 = 0;
				tmp 	|=  r_buffer[29];
				tmp      =  tmp<<8;
				tmp 	|=  r_buffer[30];
				
//               	tmp2	 = 0;
//				tmp2 	|=  r_buffer[31];
//				tmp2     =  tmp2<<8;
//				tmp2 	|=  r_buffer[32];				
                McuSend_PcReadData.TimeStampex_Couter = tmp;//<<16 | tmp2;	
				
				McuSend_PcReadData.TimeStamp[McuSend_PcReadData.TimeStampex_Couter%STAMPNUMS][0] = 
									McuSend_PcReadData.TimeStampexTmp[0];			
				McuSend_PcReadData.TimeStamp[McuSend_PcReadData.TimeStampex_Couter%STAMPNUMS][1] = 
									McuSend_PcReadData.TimeStampexTmp[1];
				McuSend_PcReadData.TimeStamp[McuSend_PcReadData.TimeStampex_Couter%STAMPNUMS][2] = 
									McuSend_PcReadData.TimeStampexTmp[2];		

				//deal the stamp //repair  180ms----9
				if((McuSend_PcReadData.TimeStampex_Couter%STAMPNUMS)>=BACKNUMS)
				{
					McuSend_PcReadData.TimeStampex[0] = 
					McuSend_PcReadData.TimeStamp[McuSend_PcReadData.TimeStampex_Couter%STAMPNUMS-BACKNUMS][0];
					McuSend_PcReadData.TimeStampex[1] = 
					McuSend_PcReadData.TimeStamp[McuSend_PcReadData.TimeStampex_Couter%STAMPNUMS-BACKNUMS][1];
					McuSend_PcReadData.TimeStampex[2] = 
					McuSend_PcReadData.TimeStamp[McuSend_PcReadData.TimeStampex_Couter%STAMPNUMS-BACKNUMS][2];
				}
				else
				{
					McuSend_PcReadData.TimeStampex[0] = 
					McuSend_PcReadData.TimeStamp[STAMPNUMS-(BACKNUMS-McuSend_PcReadData.TimeStampex_Couter%STAMPNUMS)][0];
					McuSend_PcReadData.TimeStampex[1] = 
					McuSend_PcReadData.TimeStamp[STAMPNUMS-(BACKNUMS-McuSend_PcReadData.TimeStampex_Couter%STAMPNUMS)][1];
					McuSend_PcReadData.TimeStampex[2] = 
					McuSend_PcReadData.TimeStamp[STAMPNUMS-(BACKNUMS-McuSend_PcReadData.TimeStampex_Couter%STAMPNUMS)][2];					
				}
				if(printf_flag)
				{
					for(tmpi=0;tmpi<20;tmpi++)
						printf("McuSend_PcReadData.TimeStamp[%hx][0]:x=%hx,\n",tmpi,
											McuSend_PcReadData.TimeStamp[tmpi][0]);
					
					
					printf("McuSend_PcReadData.TimeStamp[x][0]:x=%hx,\n",
											McuSend_PcReadData.TimeStampex_Couter%STAMPNUMS);	
					
					printf("McuSend_PcReadData.TimeStampex[0]:x=%hx,\n",McuSend_PcReadData.TimeStampex[0]);
				}					
					
				break;		
								
			case ID_WHEELSPEED: //0x52
		
#if 1

//				if(McuSend_PcReadData.Pc_Read_Flag == 0)
				{
					tmp      =   0;
					tmp 	|=  r_buffer[3];
					tmp      =  tmp<<8;
					tmp 	|=  r_buffer[4];				
					McuSend_PcReadData.WheelSpeed[0] = tmp;
					
					tmp      =  0;
					tmp 	|=  r_buffer[5];
					tmp      =  tmp<<8;
					tmp 	|=  r_buffer[6];				
					McuSend_PcReadData.WheelSpeed[1] = tmp;				
					
					tmp	 	 = 0;
					tmp 	|=  r_buffer[7];
					tmp      =  tmp<<8;
					tmp 	|=  r_buffer[8];				
					McuSend_PcReadData.WheelSpeed[2] = tmp;		

					tmp	 	 = 0;
					tmp 	|=  r_buffer[9];
					tmp      =  tmp<<8;
					tmp 	|=  r_buffer[10];				
					McuSend_PcReadData.WheelSpeed[3] = tmp;		

					tmp	 	 = 0;
					tmp 	|=  r_buffer[11];
					tmp      =  tmp<<8;
					tmp 	|=  r_buffer[12];				
					McuSend_PcReadData.WheelSpeed[4] = tmp;		
					
					McuSend_PcReadData.rader1_alarm_level		= r_buffer[13]&0x0f;
					McuSend_PcReadData.rader2_alarm_level		= r_buffer[13]>>4;
					McuSend_PcReadData.rader3_alarm_level		= r_buffer[14]&0x0f;
					McuSend_PcReadData.rader4_alarm_level		= r_buffer[14]>>4;
					McuSend_PcReadData.rader5_alarm_level		= r_buffer[15]&0x0f;
					McuSend_PcReadData.rader6_alarm_level		= r_buffer[15]>>4;
					McuSend_PcReadData.rader7_alarm_level		= r_buffer[16]&0x0f;
					McuSend_PcReadData.rader8_alarm_level		= r_buffer[16]>>4;
					McuSend_PcReadData.rader9_alarm_level		= r_buffer[17]&0x0f;
					McuSend_PcReadData.rader10_alarm_level		= r_buffer[17]>>4;
					McuSend_PcReadData.rader11_alarm_level		= r_buffer[18]&0x0f;
					McuSend_PcReadData.rader12_alarm_level		= r_buffer[18]>>4;
					McuSend_PcReadData.rader13_alarm_level		= r_buffer[19]&0x0f;
					McuSend_PcReadData.rader14_alarm_level		= r_buffer[19]>>4;
					McuSend_PcReadData.rader15_alarm_level		= r_buffer[20]&0x0f;
					McuSend_PcReadData.rader16_alarm_level		= r_buffer[20]>>4;	
					
					McuSend_PcReadData.McuKey 	  		  		= r_buffer[21];
					McuSend_PcReadData.parking_mode_select01 	= r_buffer[22];
					McuSend_PcReadData.parking_mode_select02 	= r_buffer[23];	

//					printf("parking_mode_select01:x=%hx\n",McuSend_PcReadData.parking_mode_select01);
//					printf("parking_mode_select02:x=%hx\n",McuSend_PcReadData.parking_mode_select02);

					
					tmp	 	 = 0;
					tmp 	|=  r_buffer[24];
					tmp      =  tmp<<8;
					tmp 	|=  r_buffer[25];				
					McuSend_PcReadData.trail_arc_radius 		= tmp;					
					
					tmp	 	 = 0;
					tmp 	|=  r_buffer[26];
					tmp      =  tmp<<8;
					tmp 	|=  r_buffer[27];				
					McuSend_PcReadData.trail_arc_angle			= tmp;
		
					McuSend_PcReadData.car_paring_status        = r_buffer[28];						
				
				}

					if(r_buffer[29]==1)
					{
						Mcu_receive_FLAG = McuOk;
						Pc_Tx_Mcu_Nums = 0;
						
						if(printf_flag_ext)
							printf("Mcu_receive_FLAG_52:x=%hx\n",Mcu_receive_FLAG);
					}
						


#endif
				
				if(printf_flag_ext)
				{
				printf("WheelSpeed[0]:x=%hx,WheelSpeed[1]:x=%hx,WheelSpeed[2]:x=%hx\n,WheelSpeed[3]:x=%hx,WheelSpeed[4]:x=%hx\n",
						McuSend_PcReadData.WheelSpeed[0],McuSend_PcReadData.WheelSpeed[1],McuSend_PcReadData.WheelSpeed[2],
				McuSend_PcReadData.WheelSpeed[3],McuSend_PcReadData.WheelSpeed[4]);
				printf("TimeStampex[0]:x=%hx,TimeStampex[1]:x=%hx\n",
						McuSend_PcReadData.TimeStampex[0],McuSend_PcReadData.TimeStampex[1]);
				
				}
			
				break;
				
			default:
				break;
		}
	}	
	else
	{	if(printf_flag)
		printf("error:f_start = %x\n", mcu_frame.f_start);
	}
	
}

void *Uart_TX_thread(void *t) 
{
//	static int cnt;
//	char i =0;//for test 
	//static int cnt2;
	McuReceive_PcWriteData.Pc_Write_Flag = 0;//for test to be del 
	
	while(1)
	{
		usleep(1000);////0
		if(((Mcu_receive_FLAG != McuOk)&&(Pc_Tx_Mcu_Nums <= 30))||(McuReceive_PcWriteData.Pc_Write_Flag == 1))
//		if(McuReceive_PcWriteData.Pc_Write_Flag == 1)
		{
					if(McuReceive_PcWriteData.Pc_Write_Flag == 1)
					{
						Pc_Tx_Mcu_Nums = 0;
						McuReceive_PcWriteData.Pc_Write_Flag = 0;
						Mcu_receive_FLAG = McuErr;
					}
					else
						Pc_Tx_Mcu_Nums++;									
					
									
					
					txBuffer[56] = 	F_START_SOC;//0x01
					txBuffer[57] = 	0x81;       
					txBuffer[58] = 	52;					
					txBuffer[59] = 	McuReceive_PcWriteData.PC_BackNear_TyreGound_X>>8;
					txBuffer[60] = 	McuReceive_PcWriteData.PC_BackNear_TyreGound_X;
					txBuffer[61] = 	McuReceive_PcWriteData.PC_BackNear_TyreGound_Y>>8;
					txBuffer[62] = 	McuReceive_PcWriteData.PC_BackNear_TyreGound_Y;
					txBuffer[63] = 	McuReceive_PcWriteData.PC_BackFar_TyreGound_X>>8;
					txBuffer[64] =  McuReceive_PcWriteData.PC_BackFar_TyreGound_X;
					txBuffer[65] = 	McuReceive_PcWriteData.PC_BackFar_TyreGound_Y>>8;
					txBuffer[66] = 	McuReceive_PcWriteData.PC_BackFar_TyreGound_Y;
					txBuffer[67] = 	McuReceive_PcWriteData.PC_Back_TyreGound_TimeStampeX>>8;
					txBuffer[68] =  McuReceive_PcWriteData.PC_Back_TyreGound_TimeStampeX;
					txBuffer[69] = 	McuReceive_PcWriteData.PC_Back_TyreGound_TimeStampeY>>8;
					txBuffer[70] = 	McuReceive_PcWriteData.PC_Back_TyreGound_TimeStampeY;
					txBuffer[71] = 	McuReceive_PcWriteData.PC_Front_Bumper_X1>>8;
					txBuffer[72] =  McuReceive_PcWriteData.PC_Front_Bumper_X1;
					txBuffer[73] = 	McuReceive_PcWriteData.PC_Front_Bumper_Y1>>8;
					txBuffer[74] = 	McuReceive_PcWriteData.PC_Front_Bumper_Y1;
					txBuffer[75] = 	McuReceive_PcWriteData.PC_Front_Bumper_X2>>8;
					txBuffer[76] = 	McuReceive_PcWriteData.PC_Front_Bumper_X2;
					txBuffer[77] = 	McuReceive_PcWriteData.PC_Front_Bumper_Y2>>8;
					txBuffer[78] = 	McuReceive_PcWriteData.PC_Front_Bumper_Y2;
					txBuffer[79] = 	McuReceive_PcWriteData.PC_Front_Bumper_X3>>8;
					txBuffer[80] = 	McuReceive_PcWriteData.PC_Front_Bumper_X3;
					txBuffer[81] = 	McuReceive_PcWriteData.PC_Front_Bumper_Y3>>8;
					txBuffer[82] = 	McuReceive_PcWriteData.PC_Front_Bumper_Y3;
					txBuffer[83] = 	McuReceive_PcWriteData.PC_Front_Bumper_X4>>8;
					txBuffer[84] =  McuReceive_PcWriteData.PC_Front_Bumper_X4;
					txBuffer[85] = 	McuReceive_PcWriteData.PC_Front_Bumper_Y4>>8;
					txBuffer[86] = 	McuReceive_PcWriteData.PC_Front_Bumper_Y4;
					txBuffer[87] = 	McuReceive_PcWriteData.PC_Front_Bumper_TimeStampeX>>8;
					txBuffer[88] =  McuReceive_PcWriteData.PC_Front_Bumper_TimeStampeX;
					txBuffer[89] = 	McuReceive_PcWriteData.PC_Front_Bumper_TimeStampeY>>8;
					txBuffer[90] = 	McuReceive_PcWriteData.PC_Front_Bumper_TimeStampeY;
					txBuffer[91] = 	McuReceive_PcWriteData.PC_Back_Bumper_X1>>8;
					txBuffer[92] =  McuReceive_PcWriteData.PC_Back_Bumper_X1;
					txBuffer[93] = 	McuReceive_PcWriteData.PC_Back_Bumper_Y1>>8;
					txBuffer[94] = 	McuReceive_PcWriteData.PC_Back_Bumper_Y1;
					txBuffer[95] = 	McuReceive_PcWriteData.PC_Back_Bumper_X2>>8;
					txBuffer[96] = 	McuReceive_PcWriteData.PC_Back_Bumper_X2;
					txBuffer[97] = 	McuReceive_PcWriteData.PC_Back_Bumper_Y2>>8;
					txBuffer[98] = 	McuReceive_PcWriteData.PC_Back_Bumper_Y2;
					txBuffer[99] = 	McuReceive_PcWriteData.PC_Back_Bumper_X3>>8;
					txBuffer[100] = 	McuReceive_PcWriteData.PC_Back_Bumper_X3;
					txBuffer[101] = 	McuReceive_PcWriteData.PC_Back_Bumper_Y3>>8;
					txBuffer[102] = 	McuReceive_PcWriteData.PC_Back_Bumper_Y3;
					txBuffer[103] = 	McuReceive_PcWriteData.PC_Back_Bumper_X4>>8;
					txBuffer[104] =  McuReceive_PcWriteData.PC_Back_Bumper_X4;
					txBuffer[105] = 	McuReceive_PcWriteData.PC_Back_Bumper_Y4>>8;
					txBuffer[106] = 	McuReceive_PcWriteData.PC_Back_Bumper_Y4;
					txBuffer[107] = McuReceive_PcWriteData.PC_Back_Bumper_TimeStampeX>>8;
					txBuffer[108] = McuReceive_PcWriteData.PC_Back_Bumper_TimeStampeX;
					txBuffer[109] = McuReceive_PcWriteData.PC_Back_Bumper_TimeStampeY>>8;
					txBuffer[110] = McuReceive_PcWriteData.PC_Back_Bumper_TimeStampeY;		
					
					txBuffer[111] = Add_Verify(&txBuffer[59],52);
					write(fd_uart, &txBuffer[56], 56);	
					usleep(27500);	//24500
					
//					txBuffer[104] = Add_Verify(&txBuffer[3],101);
//					write(fd_uart, &txBuffer[0], 105);	
//					usleep(49000);	//49000			
txBuffer[0] = 	F_START_SOC;//0x01
					txBuffer[1] = 	0x80;       
					txBuffer[2] = 	52;
					txBuffer[3] = 	McuReceive_PcWriteData.PC_CarPark_P0Point[0]>>8;
					txBuffer[4] = 	McuReceive_PcWriteData.PC_CarPark_P0Point[0];
					txBuffer[5] = 	McuReceive_PcWriteData.PC_CarPark_P0Point[1]>>8;
					txBuffer[6] = 	McuReceive_PcWriteData.PC_CarPark_P0Point[1];
					txBuffer[7] = 	McuReceive_PcWriteData.PC_CarPark_P1Point[0]>>8;
					txBuffer[8] = 	McuReceive_PcWriteData.PC_CarPark_P1Point[0];
					txBuffer[9] = 	McuReceive_PcWriteData.PC_CarPark_P1Point[1]>>8;
					txBuffer[10] = 	McuReceive_PcWriteData.PC_CarPark_P1Point[1];
					txBuffer[11] = 	McuReceive_PcWriteData.PC_CarPark_P2Point[0]>>8;
					txBuffer[12] = 	McuReceive_PcWriteData.PC_CarPark_P2Point[0];
					txBuffer[13] = 	McuReceive_PcWriteData.PC_CarPark_P2Point[1]>>8;
					txBuffer[14] = 	McuReceive_PcWriteData.PC_CarPark_P2Point[1];
					txBuffer[15] = 	McuReceive_PcWriteData.PC_CarPark_P3Point[0]>>8;
					txBuffer[16] = 	McuReceive_PcWriteData.PC_CarPark_P3Point[0];
					txBuffer[17] = 	McuReceive_PcWriteData.PC_CarPark_P3Point[1]>>8;
					txBuffer[18] = 	McuReceive_PcWriteData.PC_CarPark_P3Point[1];
					txBuffer[19] = 	McuReceive_PcWriteData.PC_ZhiXinDu;
					txBuffer[20] = 	McuReceive_PcWriteData.PC_TimeStampeX>>8;
					txBuffer[21] = 	McuReceive_PcWriteData.PC_TimeStampeX;
					txBuffer[22] = 	McuReceive_PcWriteData.PC_TimeStampeY>>8;
					txBuffer[23] = 	McuReceive_PcWriteData.PC_TimeStampeY;
					
					txBuffer[24] = 	McuReceive_PcWriteData.PC_TimeStampeZ>>8;
					txBuffer[25] = 	McuReceive_PcWriteData.PC_TimeStampeZ;							
					txBuffer[26] = 	McuReceive_PcWriteData.PC_CarFront_Ear_X>>8;
					txBuffer[27] = 	McuReceive_PcWriteData.PC_CarFront_Ear_X;
					txBuffer[28] = 	McuReceive_PcWriteData.PC_CarFront_Ear_Y>>8;
					txBuffer[29] = 	McuReceive_PcWriteData.PC_CarFront_Ear_Y;
					txBuffer[30] = 	McuReceive_PcWriteData.PC_Front_Ear_TimeStampeX>>8;
					txBuffer[31] = 	McuReceive_PcWriteData.PC_Front_Ear_TimeStampeX;
					txBuffer[32] = 	McuReceive_PcWriteData.PC_Front_Ear_TimeStampeY>>8;
					txBuffer[33] = 	McuReceive_PcWriteData.PC_Front_Ear_TimeStampeY;
					txBuffer[34] = 	McuReceive_PcWriteData.PC_CarBack_Ear_X>>8;
					txBuffer[35] = 	McuReceive_PcWriteData.PC_CarBack_Ear_X;
					txBuffer[36] = 	McuReceive_PcWriteData.PC_CarBack_Ear_Y>>8;
					txBuffer[37] = 	McuReceive_PcWriteData.PC_CarBack_Ear_Y;
					txBuffer[38] = 	McuReceive_PcWriteData.PC_Back_Ear_TimeStampeX>>8;
					txBuffer[39] =  McuReceive_PcWriteData.PC_Back_Ear_TimeStampeX;
					txBuffer[40] = 	McuReceive_PcWriteData.PC_Back_Ear_TimeStampeY>>8;
					txBuffer[41] = 	McuReceive_PcWriteData.PC_Back_Ear_TimeStampeY;
					txBuffer[42] = 	McuReceive_PcWriteData.PC_FrontNear_TyreGound_X>>8;
					txBuffer[43] =  McuReceive_PcWriteData.PC_FrontNear_TyreGound_X;
					txBuffer[44] = 	McuReceive_PcWriteData.PC_FrontNear_TyreGound_Y>>8;
					txBuffer[45] = 	McuReceive_PcWriteData.PC_FrontNear_TyreGound_Y;
					txBuffer[46] = 	McuReceive_PcWriteData.PC_FrontFar_TyreGound_X>>8;
					txBuffer[47] =  McuReceive_PcWriteData.PC_FrontFar_TyreGound_X;
					txBuffer[48] = 	McuReceive_PcWriteData.PC_FrontFar_TyreGound_Y>>8;
					txBuffer[49] = 	McuReceive_PcWriteData.PC_FrontFar_TyreGound_Y;
					txBuffer[50] = 	McuReceive_PcWriteData.PC_Front_TyreGound_TimeStampeX>>8;
					txBuffer[51] = 	McuReceive_PcWriteData.PC_Front_TyreGound_TimeStampeX;
					txBuffer[52] = 	McuReceive_PcWriteData.PC_Front_TyreGound_TimeStampeY>>8;
					txBuffer[53] = 	McuReceive_PcWriteData.PC_Front_TyreGound_TimeStampeY;
					txBuffer[54] = 	0;
					
					txBuffer[55] = Add_Verify(&txBuffer[3],52);
					write(fd_uart, txBuffer, 56);	
					usleep(27500);	//24500	





					
					
		}
	}
}


 void write_data(void)
 {
		if(((Mcu_receive_FLAG != McuOk)&&(Pc_Tx_Mcu_Nums <= 10))||(McuReceive_PcWriteData.Pc_Write_Flag == 1))
//		if(McuReceive_PcWriteData.Pc_Write_Flag == 1)
		{
					if(McuReceive_PcWriteData.Pc_Write_Flag == 1)
					{
						Pc_Tx_Mcu_Nums = 0;
						McuReceive_PcWriteData.Pc_Write_Flag = 0;
						Mcu_receive_FLAG = McuErr;
					}
					else
						Pc_Tx_Mcu_Nums++;										
					
					txBuffer[0] = 	F_START_SOC;//0x01
					txBuffer[1] = 	0x80;       
					txBuffer[2] = 	101;
					txBuffer[3] = 	McuReceive_PcWriteData.PC_CarPark_P0Point[0]>>8;
					txBuffer[4] = 	McuReceive_PcWriteData.PC_CarPark_P0Point[0];
					txBuffer[5] = 	McuReceive_PcWriteData.PC_CarPark_P0Point[1]>>8;
					txBuffer[6] = 	McuReceive_PcWriteData.PC_CarPark_P0Point[1];
					txBuffer[7] = 	McuReceive_PcWriteData.PC_CarPark_P1Point[0]>>8;
					txBuffer[8] = 	McuReceive_PcWriteData.PC_CarPark_P1Point[0];
					txBuffer[9] = 	McuReceive_PcWriteData.PC_CarPark_P1Point[1]>>8;
					txBuffer[10] = 	McuReceive_PcWriteData.PC_CarPark_P1Point[1];
					txBuffer[11] = 	McuReceive_PcWriteData.PC_CarPark_P2Point[0]>>8;
					txBuffer[12] = 	McuReceive_PcWriteData.PC_CarPark_P2Point[0];
					txBuffer[13] = 	McuReceive_PcWriteData.PC_CarPark_P2Point[1]>>8;
					txBuffer[14] = 	McuReceive_PcWriteData.PC_CarPark_P2Point[1];
					txBuffer[15] = 	McuReceive_PcWriteData.PC_CarPark_P3Point[0]>>8;
					txBuffer[16] = 	McuReceive_PcWriteData.PC_CarPark_P3Point[0];
					txBuffer[17] = 	McuReceive_PcWriteData.PC_CarPark_P3Point[1]>>8;
					txBuffer[18] = 	McuReceive_PcWriteData.PC_CarPark_P3Point[1];
					txBuffer[19] = 	McuReceive_PcWriteData.PC_ZhiXinDu;
					txBuffer[20] = 	McuReceive_PcWriteData.PC_TimeStampeX>>8;
					txBuffer[21] = 	McuReceive_PcWriteData.PC_TimeStampeX;
					txBuffer[22] = 	McuReceive_PcWriteData.PC_TimeStampeY>>8;
					txBuffer[23] = 	McuReceive_PcWriteData.PC_TimeStampeY;
					txBuffer[24] = 	McuReceive_PcWriteData.PC_CarFront_Ear_X>>8;
					txBuffer[25] = 	McuReceive_PcWriteData.PC_CarFront_Ear_X;
					txBuffer[26] = 	McuReceive_PcWriteData.PC_CarFront_Ear_Y>>8;
					txBuffer[27] = 	McuReceive_PcWriteData.PC_CarFront_Ear_Y;
					txBuffer[28] = 	McuReceive_PcWriteData.PC_Front_Ear_TimeStampeX>>8;
					txBuffer[29] = 	McuReceive_PcWriteData.PC_Front_Ear_TimeStampeX;
					txBuffer[30] = 	McuReceive_PcWriteData.PC_Front_Ear_TimeStampeY>>8;
					txBuffer[31] = 	McuReceive_PcWriteData.PC_Front_Ear_TimeStampeY;
					txBuffer[32] = 	McuReceive_PcWriteData.PC_CarBack_Ear_X>>8;
					txBuffer[33] = 	McuReceive_PcWriteData.PC_CarBack_Ear_X;
					txBuffer[34] = 	McuReceive_PcWriteData.PC_CarBack_Ear_Y>>8;
					txBuffer[35] = 	McuReceive_PcWriteData.PC_CarBack_Ear_Y;
					txBuffer[36] = 	McuReceive_PcWriteData.PC_Back_Ear_TimeStampeX>>8;
					txBuffer[37] =  McuReceive_PcWriteData.PC_Back_Ear_TimeStampeX;
					txBuffer[38] = 	McuReceive_PcWriteData.PC_Back_Ear_TimeStampeY>>8;
					txBuffer[39] = 	McuReceive_PcWriteData.PC_Back_Ear_TimeStampeY;
					txBuffer[40] = 	McuReceive_PcWriteData.PC_FrontNear_TyreGound_X>>8;
					txBuffer[41] =  McuReceive_PcWriteData.PC_FrontNear_TyreGound_X;
					txBuffer[42] = 	McuReceive_PcWriteData.PC_FrontNear_TyreGound_Y>>8;
					txBuffer[43] = 	McuReceive_PcWriteData.PC_FrontNear_TyreGound_Y;
					txBuffer[44] = 	McuReceive_PcWriteData.PC_FrontFar_TyreGound_X>>8;
					txBuffer[45] =  McuReceive_PcWriteData.PC_FrontFar_TyreGound_X;
					txBuffer[46] = 	McuReceive_PcWriteData.PC_FrontFar_TyreGound_Y>>8;
					txBuffer[47] = 	McuReceive_PcWriteData.PC_FrontFar_TyreGound_Y;
					txBuffer[48] = 	McuReceive_PcWriteData.PC_Front_TyreGound_TimeStampeX>>8;
					txBuffer[49] = 	McuReceive_PcWriteData.PC_Front_TyreGound_TimeStampeX;
					txBuffer[50] = 	McuReceive_PcWriteData.PC_Front_TyreGound_TimeStampeY>>8;
					txBuffer[51] = 	McuReceive_PcWriteData.PC_Front_TyreGound_TimeStampeY;
					txBuffer[52] = 	McuReceive_PcWriteData.PC_BackNear_TyreGound_X>>8;
					txBuffer[53] = 	McuReceive_PcWriteData.PC_BackNear_TyreGound_X;
					txBuffer[54] = 	McuReceive_PcWriteData.PC_BackNear_TyreGound_Y>>8;
					txBuffer[55] = 	McuReceive_PcWriteData.PC_BackNear_TyreGound_Y;
					txBuffer[56] = 	McuReceive_PcWriteData.PC_BackFar_TyreGound_X>>8;
					txBuffer[57] =  McuReceive_PcWriteData.PC_BackFar_TyreGound_X;
					txBuffer[58] = 	McuReceive_PcWriteData.PC_BackFar_TyreGound_Y>>8;
					txBuffer[59] = 	McuReceive_PcWriteData.PC_BackFar_TyreGound_Y;
					txBuffer[60] = 	McuReceive_PcWriteData.PC_Back_TyreGound_TimeStampeX>>8;
					txBuffer[61] =  McuReceive_PcWriteData.PC_Back_TyreGound_TimeStampeX;
					txBuffer[62] = 	McuReceive_PcWriteData.PC_Back_TyreGound_TimeStampeY>>8;
					txBuffer[63] = 	McuReceive_PcWriteData.PC_Back_TyreGound_TimeStampeY;
					txBuffer[64] = 	McuReceive_PcWriteData.PC_Front_Bumper_X1>>8;
					txBuffer[65] =  McuReceive_PcWriteData.PC_Front_Bumper_X1;
					txBuffer[66] = 	McuReceive_PcWriteData.PC_Front_Bumper_Y1>>8;
					txBuffer[67] = 	McuReceive_PcWriteData.PC_Front_Bumper_Y1;
					txBuffer[68] = 	McuReceive_PcWriteData.PC_Front_Bumper_X2>>8;
					txBuffer[69] = 	McuReceive_PcWriteData.PC_Front_Bumper_X2;
					txBuffer[70] = 	McuReceive_PcWriteData.PC_Front_Bumper_Y2>>8;
					txBuffer[71] = 	McuReceive_PcWriteData.PC_Front_Bumper_Y2;
					txBuffer[72] = 	McuReceive_PcWriteData.PC_Front_Bumper_X3>>8;
					txBuffer[73] = 	McuReceive_PcWriteData.PC_Front_Bumper_X3;
					txBuffer[74] = 	McuReceive_PcWriteData.PC_Front_Bumper_Y3>>8;
					txBuffer[75] = 	McuReceive_PcWriteData.PC_Front_Bumper_Y3;
					txBuffer[76] = 	McuReceive_PcWriteData.PC_Front_Bumper_X4>>8;
					txBuffer[77] =  McuReceive_PcWriteData.PC_Front_Bumper_X4;
					txBuffer[78] = 	McuReceive_PcWriteData.PC_Front_Bumper_Y4>>8;
					txBuffer[79] = 	McuReceive_PcWriteData.PC_Front_Bumper_Y4;
					txBuffer[80] = 	McuReceive_PcWriteData.PC_Front_Bumper_TimeStampeX>>8;
					txBuffer[81] =  McuReceive_PcWriteData.PC_Front_Bumper_TimeStampeX;
					txBuffer[82] = 	McuReceive_PcWriteData.PC_Front_Bumper_TimeStampeY>>8;
					txBuffer[83] = 	McuReceive_PcWriteData.PC_Front_Bumper_TimeStampeY;
					txBuffer[84] = 	McuReceive_PcWriteData.PC_Back_Bumper_X1>>8;
					txBuffer[85] =  McuReceive_PcWriteData.PC_Back_Bumper_X1;
					txBuffer[86] = 	McuReceive_PcWriteData.PC_Back_Bumper_Y1>>8;
					txBuffer[87] = 	McuReceive_PcWriteData.PC_Back_Bumper_Y1;
					txBuffer[88] = 	McuReceive_PcWriteData.PC_Back_Bumper_X2>>8;
					txBuffer[89] = 	McuReceive_PcWriteData.PC_Back_Bumper_X2;
					txBuffer[90] = 	McuReceive_PcWriteData.PC_Back_Bumper_Y2>>8;
					txBuffer[91] = 	McuReceive_PcWriteData.PC_Back_Bumper_Y2;
					txBuffer[92] = 	McuReceive_PcWriteData.PC_Back_Bumper_X3>>8;
					txBuffer[93] = 	McuReceive_PcWriteData.PC_Back_Bumper_X3;
					txBuffer[94] = 	McuReceive_PcWriteData.PC_Back_Bumper_Y3>>8;
					txBuffer[95] = 	McuReceive_PcWriteData.PC_Back_Bumper_Y3;
					txBuffer[96] = 	McuReceive_PcWriteData.PC_Back_Bumper_X4>>8;
					txBuffer[97] =  McuReceive_PcWriteData.PC_Back_Bumper_X4;
					txBuffer[98] = 	McuReceive_PcWriteData.PC_Back_Bumper_Y4>>8;
					txBuffer[99] = 	McuReceive_PcWriteData.PC_Back_Bumper_Y4;
					txBuffer[100] = McuReceive_PcWriteData.PC_Back_Bumper_TimeStampeX>>8;
					txBuffer[101] = McuReceive_PcWriteData.PC_Back_Bumper_TimeStampeX;
					txBuffer[102] = McuReceive_PcWriteData.PC_Back_Bumper_TimeStampeY>>8;
					txBuffer[103] = McuReceive_PcWriteData.PC_Back_Bumper_TimeStampeY;
					txBuffer[104] = Add_Verify(&txBuffer[3],101);
					write(fd_uart, txBuffer, 105);	
					usleep(105000);	//100
		}
 
 }






unsigned char Add_Verify(unsigned char *p,unsigned char length)
{
	unsigned char  i,tmp=0;

	for(i=0;i<length;i++)
	{
		tmp += p[i];
	}
	tmp = 0-tmp;//~tmp;
	return tmp;
}
