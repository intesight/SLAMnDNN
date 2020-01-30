#ifndef __UART_TO_MCU_H__
#define __UART_TO_MCU_H__

#include <semaphore.h>
#include <pthread.h>
#include <sys/time.h>
#include "unistd.h"

#define STAMPNUMS 10 //20
#define BACKNUMS  9  //9--20ms//18---10ms

typedef struct {
	unsigned char 	f_start;
	unsigned char	  id_flag;
	unsigned char  	d_len;
	unsigned char	  *data;
	unsigned char	  xor_flag;
}mcu_soc_frame_t;

typedef struct PcReadData
{
	char Pc_Read_Flag ;					//PC读数据标志   1：读数据刷新  0：读数据不刷新
	char rader1_alarm_level;			//超声波1号的报警等级
	char rader2_alarm_level;			//超声波2号的报警等级
	char rader3_alarm_level;			//超声波3号的报警等级
	char rader4_alarm_level;			//超声波4号的报警等级
	char rader5_alarm_level;			//超声波5号的报警等级
	char rader6_alarm_level;			//超声波6号的报警等级
	char rader7_alarm_level;			//超声波7号的报警等级
	char rader8_alarm_level;			//超声波8号的报警等级
	char rader9_alarm_level;			//超声波9号的报警等级
	char rader10_alarm_level;			//超声波10号的报警等级
	char rader11_alarm_level;			//超声波11号的报警等级
	char rader12_alarm_level;			//超声波12号的报警等级
	char rader13_alarm_level;			//超声波13号的报警等级
	char rader14_alarm_level;			//超声波14号的报警等级
	char rader15_alarm_level;			//超声波15号的报警等级
	char rader16_alarm_level;			//超声波16号的报警等级

	short int parking_rect_point0_x;	//显示车位角点坐标X1
	short int parking_rect_point0_y;	//显示车位角点坐标Y1
	short int parking_rect_point1_x;	//显示车位角点坐标X2
	short int parking_rect_point1_y;	//显示车位角点坐标Y2
	short int parking_rect_point2_x;	//显示车位角点坐标X3
	short int parking_rect_point2_y;	//显示车位角点坐标Y3
	short int parking_rect_point3_x;	//显示车位角点坐标X4
	short int parking_rect_point3_y;	//显示车位角点坐标Y4

	//short int parking_distance_start_x;
	//short int parking_distance_start_y;
	//short int parking_distance_end_x;
	//short int parking_distance_end_y;
	//short int parking_distance_depth;

	char car_paring_status;				//泊车状态
	short int TimeStampex[3]; 			//TimeStampex[0]：time stampex   TimeStampex[1]：time stampey  TimeStampex[2]：time stampey
	unsigned int TimeStampex_Couter;
	short int TimeStamp[STAMPNUMS][3];
	short int TimeStampexTmp[3];
	
	//int trail_line1_start_x;
	//int trail_line1_start_y;
	//int trail_line1_end_x;
	//int trail_line1_end_y;
	//int trail_line2_start_x;
	//int trail_line2_start_y;
	//int trail_line2_end_x;
	//int trail_line2_end_y;
	//int trail_arc_start_x;
	//int trail_arc_start_y;
	//int trail_arc_end_x;
	//int trail_arc_end_y;
	short int trail_arc_radius ;		//泊车轨迹半径
	short int trail_arc_angle;			//泊车轨迹弧度
	char  parking_mode_select01;		//泊车模式选择显示01//key1
	char  parking_mode_select02;		//泊车模式选择显示02//key2
	char gear_status_actual;			//实际档位信息
	char gear_status_pre;				//期望档位信息
	char McuKey;						//转向灯
	short int WheelSpeed[5];  			//WheelSpeed[0]:左前轮轮速WheelSpeed[1]:右前轮轮速WheelSpeed[2]:左后轮轮速
										//WheelSpeed[3]:右后轮轮速WheelSpeed[4]:前轮转角
}PCREADDATA;

typedef struct PcWriteData
{
	unsigned char Pc_Write_Flag ;	//PC写数据标志     
	/**		   bit7.6.5.4.3.2.1.0					
				  ｜｜｜｜｜｜｜｜--- 第一组数据发送标志位 1：执行写数据  0：不执行写数据
				  ｜｜｜｜｜｜｜—---- 第二组数据发送标志位 1：执行写数据  0：不执行写数据
				  ｜｜｜｜｜｜—----—— 第三组数据发送标志位 1：执行写数据  0：不执行写数据
				  ｜｜｜｜｜—------—— 第四组数据发送标志位 1：执行写数据  0：不执行写数据
				  ｜｜｜｜—--------—— 第五组数据发送标志位 1：执行写数据  0：不执行写数据
				  ｜｜｜—----------—— 第六组数据发送标志位 1：执行写数据  0：不执行写数据
				  ｜｜—------------—— 第七组数据发送标志位 1：执行写数据  0：不执行写数据
				  ｜—--------------—— 第八组数据发送标志位 1：执行写数据  0：不执行写数据
	**********************************************************/
	short int PC_CarPark_P0Point[2];  //P0点坐标（x,y）x1:PC_CarPark_P0Point[0]  y1:PC_CarPark_P0Point[1]
	short int PC_CarPark_P1Point[2];  //P1点坐标（x,y）x2:PC_CarPark_P1Point[0]  y2:PC_CarPark_P1Point[1]
	short int PC_CarPark_P2Point[2];  //P2点坐标（x,y）x3:PC_CarPark_P2Point[0]  y3:PC_CarPark_P2Point[1]
	short int PC_CarPark_P3Point[2];  //P3点坐标（x,y）x4:PC_CarPark_P3Point[0]  y4:PC_CarPark_P3Point[1]
	char PC_ZhiXinDu;      //置信度
	short int PC_TimeStampeX;//车位识别TimeStampeX
	short int PC_TimeStampeY;//车位识别TimeStampeY
	short int PC_TimeStampeZ;//车位识别TimeStampeZ
	
	short int PC_CarPositionX;//PC端车位识别X 
	short int PC_CarPositionY;//PC端车位识别Y 
	short int PC_CarPositionZ;//PC端车位识别Z 	
	
	
	short int PC_CarFront_Ear_X;//车位前侧耳朵外侧点坐标值X
	short int PC_CarFront_Ear_Y;//车位前侧耳朵外侧点坐标值Y
//	short int PC_Front_Ear_TimeStampeX;//前耳朵识别Time stampeX
//	short int PC_Front_Ear_TimeStampeY;//前耳朵识别Time stampeY
	short int PC_CarBack_Ear_X;//车位后侧耳朵外侧点坐标值X
	short int PC_CarBack_Ear_Y;//车位后侧耳朵外侧点坐标值y
//	short int PC_Back_Ear_TimeStampeX;//后耳朵识别Time stampeX
//	short int PC_Back_Ear_TimeStampeY;//后耳朵识别Time stampeY
	short int PC_FrontNear_TyreGound_X;//前侧近端轮胎接地点坐标值X
	short int PC_FrontNear_TyreGound_Y;//前侧近端轮胎接地点坐标值Y
	short int PC_FrontFar_TyreGound_X;//前侧远端轮胎接地点坐标值X
	short int PC_FrontFar_TyreGound_Y;//前侧远端轮胎接地点坐标值Y
//	short int PC_Front_TyreGound_TimeStampeX;//前侧轮胎接地点识别Time stampeX
//	short int PC_Front_TyreGound_TimeStampeY;//前侧轮胎接地点识别Time stampeX

	short int PC_BackNear_TyreGound_X;//后侧近端轮胎接地点坐标值X
	short int PC_BackNear_TyreGound_Y;//后侧近端轮胎接地点坐标值Y
	short int PC_BackFar_TyreGound_X;//后侧远端轮胎接地点坐标值X
	short int PC_BackFar_TyreGound_Y;//后侧远端轮胎接地点坐标值Y
//	short int PC_Back_TyreGound_TimeStampeX;//后侧轮胎接地点识别Time stampeX
//	short int PC_Back_TyreGound_TimeStampeY;//后侧轮胎接地点识别Time stampeX

	short int PC_Front_Bumper_X1;//车位前侧保险杠区域坐标X1
	short int PC_Front_Bumper_Y1;//车位前侧保险杠区域坐标Y1
	short int PC_Front_Bumper_X2;//车位前侧保险杠区域坐标X2
	short int PC_Front_Bumper_Y2;//车位前侧保险杠区域坐标Y2
	short int PC_Front_Bumper_X3;//车位前侧保险杠区域坐标X3
	short int PC_Front_Bumper_Y3;//车位前侧保险杠区域坐标Y3
	short int PC_Front_Bumper_X4;//车位前侧保险杠区域坐标X4
	short int PC_Front_Bumper_Y4;//车位前侧保险杠区域坐标Y4

	short int PC_Front_Bumper_TimeStampeX;//前侧保险杠识别Time stampeX
	short int PC_Front_Bumper_TimeStampeY;//前侧保险杠识别Time stampeY
	short int PC_Front_Bumper_TimeStampeZ;//前侧保险杠识别Time stampeY

	short int PC_Back_Bumper_X1;//车位后侧保险杠区域坐标X1
	short int PC_Back_Bumper_Y1;//车位后侧保险杠区域坐标Y1
	short int PC_Back_Bumper_X2;//车位后侧保险杠区域坐标X2
	short int PC_Back_Bumper_Y2;//车位后侧保险杠区域坐标Y2
	short int PC_Back_Bumper_X3;//车位后侧保险杠区域坐标X3
	short int PC_Back_Bumper_Y3;//车位后侧保险杠区域坐标Y3
	short int PC_Back_Bumper_X4;//车位后侧保险杠区域坐标X4
	short int PC_Back_Bumper_Y4;//车位后侧保险杠区域坐标Y4

	short int PC_Back_Bumper_TimeStampeX;//后侧保险杠识别Time stampeX
	short int PC_Back_Bumper_TimeStampeY;//后侧保险杠识别Time stampeY
	short int PC_Back_Bumper_TimeStampeZ;//后侧保险杠识别Time stampeY
}PCWRITEDATA;

PCREADDATA 	* get_data_from_mcu(void);
void write_data_into_mcu(PCWRITEDATA *p,unsigned char Write_Flag);


void write_data(void);

#define F_START_MCU			    0x10
#define F_START_SOC			    0x01

#define ID_HANDSSHAKE_MCU	    0x10
#define ID_HANDSSHAKE_SOC	    0x01
#define ID_HEARTBEAT_MCU        0x20
#define ID_HEARTBEAT_SOC        0x02
#define ID_ULTRASONIC_ALARM	    0x22
//#define ID_PARKING_DISTANCE     0x26
#define ID_CAR_STATUS		    0x28
#define ID_TRAIL_LINI1          0x2A
#define ID_TRAIL_ARC      	    0x2B
#define ID_TRAIL_LINI2      	0x2C
#define ID_KEY_MCU        		0xC2	
#define ID_KEY_SOC        		0x30
#define ID_KEY_SOC        		0x30
#define ID_GEAR_MCU				0x31
#define ID_PARK_RECT_POINT1		0x26
#define ID_PARK_RECT_POINT2		0x27

//below is added by che
#define ID_WHEELSPEED           0X52
#define ID_TIMESTAMPEX          0X50  //TimeStampex
extern pthread_mutex_t  mcu_data_mutex; 
extern PCREADDATA 		McuSend_PcReadData,McuSend_PcReadData_Pc;
extern PCWRITEDATA 	McuReceive_PcWriteData;
//up is added by che



#ifdef __cplusplus
extern "C" {
#endif
 void *Uart_meg_thread(void *t);
 void *Uart_TX_thread(void *t) ;
 int uart_thread_create(void);

#ifdef __cplusplus
}
#endif



#endif
