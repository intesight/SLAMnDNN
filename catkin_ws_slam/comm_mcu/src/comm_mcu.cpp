#include "comm_mcu/mcu_msg.h"
//#include "dnn_lot/parking_lots.h"
#include "message/radar_msg.h"

#include "uart_to_mcu.h"
#include "lsdLocationMsg.h"

#include "parkinggo/parkinggo_common.h"

// #include "parkinggo_common.h"

#include <opencv2/videoio.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <opencv2/core/utility.hpp>
// #include <opencv2/tracking.hpp>
#include <ros/ros.h>

using namespace cv;
using namespace std;
using namespace log4cxx;

// extern timeval radar_begin_tm;
extern timeval radar_begin_tm;
struct timeval radar_end_tm;

int radar_timedelay = 10; //10ms radar time delat

#define __PUBLISH_SLAM_LOCATION__ 1
#define __DEBUG_PRINT_TIME_POSI__ 1

enum MCU_SEND_DATA_TYPE
{
    MCU_DATA_LOTS          = 0x01,
    MCU_DATA_VEHICLE_FRONT = 0x02,
    MCU_DATA_VEHICLE_BACK  = 0x04,
    MCU_DATA_SLAM_LOCATION = 0X08
};

static Mat get_rawdata(Mat& dst1, Mat& dst2, Mat& dst3, Mat& dst4, int& pthread_stitch_flag);
static void publish_radar(PCREADDATA*& data, vector< vector<int> >& worldlots, ros::Publisher& pub_radar);
static void publish_mcu(PCREADDATA*& data, vector< vector<int> >& worldlots, ros::Publisher& pub_mcu);
static void printtime(timeval radar_begin_tm, char namestring[20], int radar_posi[3], int GPS_posi[3]);

#if __DNN_PARKING_POSITION__
static void send_parking_position(const dnn_lot::parking_lots& msg);
#endif

#if __PUBLISH_SLAM_LOCATION__
static void send_slam_car_location(const lsd_slam_core::lsdLocationMsgConstPtr& slam_location_msg);

#endif

extern PCREADDATA 	McuSend_PcReadData;
extern PCWRITEDATA 	McuReceive_PcWriteData;	

static PCREADDATA* uart_data = (PCREADDATA*)malloc(sizeof(PCREADDATA));

static vector< vector<int> > worldlots;
FILE* txt = NULL;

static int read_data_flag = 0;

static PARKINGGO_CONFIG parkinggo_configs;

static LoggerPtr logger_parkinggo(Logger::getLogger("comm_mcu"));

#if __DNN_PARKING_POSITION__
void dnn_lots_callback(const dnn_lot::parking_lots& msg)
{
    LOG4CXX_DEBUG(logger_parkinggo, "dnn_lots_callback: Lots message received");
    send_parking_position(msg);
}
#endif

#if __PUBLISH_SLAM_LOCATION__

void slam_location_callback(const lsd_slam_core::lsdLocationMsgConstPtr& slam_location_msg)
{
    LOG4CXX_DEBUG(logger_parkinggo, "slam_location_callback: slam location message received");
    send_slam_car_location(slam_location_msg);
}
#endif

int main(int argc, char** argv)
{
    set_cpu(logger_parkinggo, CPU_ID_COMM_MCU);

    load_config(logger_parkinggo, parkinggo_configs);
    
    LOG4CXX_INFO(logger_parkinggo, "Comm MCU start!");

    // mcu communications ros module
    ros::init(argc, argv, "comm_mcu");

    ros::NodeHandle nh;
    // ros::Subscriber sub = nh.subscribe("dnn_lots", 10, dnn_lots_callback);

 #if __PUBLISH_SLAM_LOCATION__
    ros::Subscriber slam_sub = nh.subscribe("lsd_slam/lsd_location", 10, slam_location_callback);
    printf("cheng debug: lsd slam \n");

 #endif
    ros::Publisher pub_radar = nh.advertise<ttmsg::radar_msg>("radar_msg", 10);
    ros::Publisher pub_mcu = nh.advertise<comm_mcu::mcu_msg>("mcu_msg", 10);

    if (parkinggo_configs.mcu_enable)
    {
        int uart_rtn = uart_thread_create();
        if(uart_rtn != 0 )
            LOG4CXX_ERROR (logger_parkinggo, "Open uart error.");

        PCWRITEDATA data_uart_write = {0};
	    usleep(10000);
    }

    ros::Rate loop_rate(50);

    int radar_test_x = 0;
    int radar_test_y = 0;

#if __DEBUG_PRINT_TIME_POSI__  
    txt = fopen("radardata.txt", "w");
    fprintf(txt, "time, radar_x, radar_y, radar_z, GPS_x, GPS_y, GPS_z \n");
#endif
    while(ros::ok())
    {
        double t = (double)cvGetTickCount();
        
        LOG4CXX_TRACE(logger_parkinggo, "main pthread: start >>>>>>>>>>>>>>>>>>>>>>>>>>");      
        LOG4CXX_TRACE(logger_parkinggo, "main pthread: run once time start");

        gettimeofday(&radar_begin_tm, NULL);

        // char namestring[20] = "time_StartRadar";
        // printtime(radar_begin_tm,namestring);

        // uart_data->car_paring_status = 1;
        // uart_data->TimeStampex[0] = radar_test_x;
        // uart_data->TimeStampex[1] = radar_test_y;
        // uart_data->TimeStampex[2] = 15;

        // uart_data->GPS_x = radar_test_x + 1;
        // uart_data->GPS_y = radar_test_y + 1;
        // uart_data->GPS_z = 15;

        // radar_test_x = radar_test_x + 10;
        // radar_test_y = radar_test_y + 20;

        uart_data = get_data_from_mcu();
        cout<<"read radar data\n"<<endl; 
        gettimeofday(&radar_end_tm, NULL);

      //  publish_radar(uart_data, worldlots, pub_radar);
        publish_mcu(uart_data, worldlots, pub_mcu);
 
        LOG4CXX_TRACE(logger_parkinggo, "main pthread: run once time finish");
        LOG4CXX_TRACE(logger_parkinggo, "main pthread: end <<<<<<<<<<<<<<<<<<<<<<<<<<<");

        loop_rate.sleep();
        // ros::spinOnce(); 

        double t_uart = (double) cvGetTickCount() - t;
        printf("cheng debug: comm_mcu pthread: run once time = %f ms\n" , t_uart/(cvGetTickFrequency()*1000));

    }
#if __DEBUG_PRINT_TIME_POSI__  
    fclose(txt);
#endif
    LOG4CXX_INFO(logger_parkinggo, "Comm MCU finish!");

    return 0;
}

static void printtime(timeval radar_begin_tm, char namestring[20], int radar_posi[3], int GPS_posi[3])
{
    struct tm *tmStartRadar;
    char time_StartRadar[30], usec_StartRadar[6];
    tmStartRadar = localtime(&radar_begin_tm.tv_sec);
    strftime(time_StartRadar, 30, "%Y%m%d-%H%M%S",tmStartRadar);
    std::cout << namestring << time_StartRadar;
    std::ostringstream ss;
    ss << std::setw(3) << std::setfill('0') << (int) radar_begin_tm.tv_usec / 1000;
    sprintf(usec_StartRadar, "%s", ss.str().c_str());

    std::cout << ":" << usec_StartRadar << std::endl;

    fprintf(txt,"%s:%s, %d, %d, %d, %d, %d, %d \n",time_StartRadar, usec_StartRadar, radar_posi[0], radar_posi[1], radar_posi[2], GPS_posi[0], GPS_posi[1],GPS_posi[2]);
}

 #if __PUBLISH_SLAM_LOCATION__

void send_slam_car_location(const lsd_slam_core::lsdLocationMsgConstPtr& slam_location_msg)
{

    PCWRITEDATA data_uart_write = {0};

    data_uart_write.PC_TimeStampeX = slam_location_msg->location_x[0];
    data_uart_write.PC_TimeStampeY = slam_location_msg->location_y[0];
    data_uart_write.PC_TimeStampeZ = slam_location_msg->angle[0];
    data_uart_write.PC_CarPositionX = slam_location_msg->location_x[1];
    data_uart_write.PC_CarPositionY = slam_location_msg->location_y[1];
    data_uart_write.PC_CarPositionZ = slam_location_msg->angle[1];

    LOG4CXX_DEBUG(logger_parkinggo, "PC_CarPositionX = " << data_uart_write.PC_CarPositionX);
    LOG4CXX_DEBUG(logger_parkinggo, "PC_CarPositionY = " << data_uart_write.PC_CarPositionY);
    LOG4CXX_DEBUG(logger_parkinggo, "PC_CarPositionZ = " << data_uart_write.PC_CarPositionZ);

    LOG4CXX_DEBUG(logger_parkinggo, "PC_TimeStampeX = " << data_uart_write.PC_TimeStampeX);
    LOG4CXX_DEBUG(logger_parkinggo, "PC_TimeStampeY = " << data_uart_write.PC_TimeStampeY);
    LOG4CXX_DEBUG(logger_parkinggo, "PC_TimeStampeZ = " << data_uart_write.PC_TimeStampeZ);

    printf("cheng debug:PC_CarPositionX = %d \n ", data_uart_write.PC_CarPositionX);
    printf("cheng debug:PC_CarPositionY = %d \n ", data_uart_write.PC_CarPositionY);
    printf("cheng debug:PC_CarPositionZ = %d \n ", data_uart_write.PC_CarPositionZ);

    double t_write_mcu_start = (double)cvGetTickCount();

    write_data_into_mcu(&data_uart_write, MCU_DATA_SLAM_LOCATION);

    double t_write_mcu = (double)cvGetTickCount() - t_write_mcu_start;

    LOG4CXX_DEBUG(logger_parkinggo, "run write_data_into_mcu time = " << t_write_mcu/(cvGetTickFrequency()*1000) << " ms");
}

#endif

#if __DNN_PARKING_POSITION__
void send_parking_position(const dnn_lot::parking_lots& msg)
{
    struct tm current_t;
    struct timeval current_tv;

    PCWRITEDATA data_uart_write = {0};
    data_uart_write.PC_CarPark_P0Point[0] = msg.lots[0].p0_x;
    data_uart_write.PC_CarPark_P0Point[1] = msg.lots[0].p0_y;
    data_uart_write.PC_CarPark_P1Point[0] = msg.lots[0].p1_x;
    data_uart_write.PC_CarPark_P1Point[1] = msg.lots[0].p1_y;
    data_uart_write.PC_CarPark_P2Point[0] = msg.lots[0].p2_x;
    data_uart_write.PC_CarPark_P2Point[1] = msg.lots[0].p2_y;
    data_uart_write.PC_CarPark_P3Point[0] = msg.lots[0].p3_x;
    data_uart_write.PC_CarPark_P3Point[1] = msg.lots[0].p3_y;

    data_uart_write.PC_TimeStampeX = msg.coordX;
    data_uart_write.PC_TimeStampeY = msg.coordY;
    data_uart_write.PC_TimeStampeZ = msg.coordZ;

    LOG4CXX_DEBUG(logger_parkinggo, "PC_CarPark_P0Point[0] = " << data_uart_write.PC_CarPark_P0Point[0]);
    LOG4CXX_DEBUG(logger_parkinggo, "PC_CarPark_P0Point[1] = " << data_uart_write.PC_CarPark_P0Point[1]);
    LOG4CXX_DEBUG(logger_parkinggo, "PC_CarPark_P1Point[0] = " << data_uart_write.PC_CarPark_P1Point[0]);
    LOG4CXX_DEBUG(logger_parkinggo, "PC_CarPark_P1Point[1] = " << data_uart_write.PC_CarPark_P1Point[1]);
    LOG4CXX_DEBUG(logger_parkinggo, "PC_CarPark_P2Point[0] = " << data_uart_write.PC_CarPark_P2Point[0]);
    LOG4CXX_DEBUG(logger_parkinggo, "PC_CarPark_P2Point[1] = " << data_uart_write.PC_CarPark_P2Point[1]);
    LOG4CXX_DEBUG(logger_parkinggo, "PC_CarPark_P3Point[0] = " << data_uart_write.PC_CarPark_P3Point[0]);
    LOG4CXX_DEBUG(logger_parkinggo, "PC_CarPark_P3Point[1] = " << data_uart_write.PC_CarPark_P3Point[1]);

    LOG4CXX_DEBUG(logger_parkinggo, "PC_TimeStampeX = " << data_uart_write.PC_TimeStampeX);
    LOG4CXX_DEBUG(logger_parkinggo, "PC_TimeStampeY = " << data_uart_write.PC_TimeStampeY);
    LOG4CXX_DEBUG(logger_parkinggo, "PC_TimeStampeZ = " << data_uart_write.PC_TimeStampeZ);

    // usleep(10000);
    double t_write_mcu_start = (double)cvGetTickCount();

    write_data_into_mcu(&data_uart_write, MCU_DATA_LOTS);

    double t_write_mcu = (double)cvGetTickCount() - t_write_mcu_start;

    LOG4CXX_DEBUG(logger_parkinggo, "run write_data_into_mcu time = " << t_write_mcu/(cvGetTickFrequency()*1000) << " ms");
}
#endif

void publish_radar(PCREADDATA*& data, vector< vector<int> >& worldlots, ros::Publisher& pub_radar)
{
    ttmsg::radar_msg radar_msg; 

    LOG4CXX_TRACE(logger_parkinggo, "worldlots.size() = " << worldlots.size() );

    for (int i=0; i<worldlots.size(); i++)
    {
        radar_msg.data[i].p0_x = worldlots[i][0]; 
        radar_msg.data[i].p0_y = worldlots[i][1]; 
        radar_msg.data[i].p1_x = worldlots[i][2]; 
        radar_msg.data[i].p1_y = worldlots[i][3]; 
        radar_msg.data[i].p2_x = worldlots[i][4];
        radar_msg.data[i].p2_y = worldlots[i][5];
        radar_msg.data[i].p3_x = worldlots[i][6];
        radar_msg.data[i].p3_y = worldlots[i][7];

        LOG4CXX_TRACE(logger_parkinggo, "p0_x = " <<  radar_msg.data[i].p0_x);
        LOG4CXX_TRACE(logger_parkinggo, "p0_y = " <<  radar_msg.data[i].p0_y);
        LOG4CXX_TRACE(logger_parkinggo, "p1_x = " <<  radar_msg.data[i].p1_x);
        LOG4CXX_TRACE(logger_parkinggo, "p1_y = " <<  radar_msg.data[i].p1_y);

        LOG4CXX_TRACE(logger_parkinggo, "p2_x = " <<  radar_msg.data[i].p2_x);
        LOG4CXX_TRACE(logger_parkinggo, "p2_y = " <<  radar_msg.data[i].p2_y);
        LOG4CXX_TRACE(logger_parkinggo, "p3_x = " <<  radar_msg.data[i].p3_x);
        LOG4CXX_TRACE(logger_parkinggo, "p4_y = " <<  radar_msg.data[i].p3_y);
    }

    radar_msg.radar1_alarm_level = data->rader1_alarm_level;
    radar_msg.radar2_alarm_level = data->rader2_alarm_level;
    radar_msg.radar3_alarm_level = data->rader3_alarm_level;
    radar_msg.radar4_alarm_level = data->rader4_alarm_level;
    radar_msg.radar5_alarm_level = data->rader5_alarm_level;
    radar_msg.radar6_alarm_level = data->rader6_alarm_level;
    radar_msg.radar7_alarm_level = data->rader7_alarm_level;
    radar_msg.radar8_alarm_level = data->rader8_alarm_level;
    radar_msg.radar9_alarm_level = data->rader9_alarm_level;
    radar_msg.radar10_alarm_level = data->rader10_alarm_level;
    radar_msg.radar11_alarm_level = data->rader11_alarm_level;
    radar_msg.radar12_alarm_level = data->rader12_alarm_level;
    radar_msg.radar13_alarm_level = data->rader13_alarm_level;
    radar_msg.radar14_alarm_level = data->rader14_alarm_level;
    radar_msg.radar15_alarm_level = data->rader15_alarm_level;
    radar_msg.radar16_alarm_level = data->rader16_alarm_level;

    radar_msg.parking_mode_select = data->parking_mode_select01;
    radar_msg.parking_mode_ok = data->parking_mode_select02;
    radar_msg.gear_status_actual = data->gear_status_actual;
    radar_msg.gear_status_pre = data->gear_status_pre;
    radar_msg.auto_parking_status = data->car_paring_status;

    radar_msg.parking_rect_point0_x = data->parking_rect_point0_x;
    radar_msg.parking_rect_point0_y = data->parking_rect_point0_y;
    radar_msg.parking_rect_point1_x = data->parking_rect_point1_x;
    radar_msg.parking_rect_point1_y = data->parking_rect_point1_y;
    radar_msg.parking_rect_point2_x = data->parking_rect_point2_x;
    radar_msg.parking_rect_point2_y = data->parking_rect_point2_y; 
    radar_msg.parking_rect_point3_x = data->parking_rect_point3_x;
    radar_msg.parking_rect_point3_y = data->parking_rect_point3_y; 

    radar_msg.trail_arc_radius = data->trail_arc_radius;
    radar_msg.trail_arc_angle = data->trail_arc_angle;

    pub_radar.publish(radar_msg);
}

void publish_mcu(PCREADDATA*& data, vector< vector<int> >& worldlots, ros::Publisher& pub_mcu)
{
    comm_mcu::mcu_msg msg; 

    LOG4CXX_TRACE(logger_parkinggo, "worldlots.size() = " << worldlots.size() );

    for (int i=0; i<worldlots.size(); i++)
    {
        msg.lots[i].p0_x = worldlots[i][0]; 
        msg.lots[i].p0_y = worldlots[i][1]; 
        msg.lots[i].p1_x = worldlots[i][2]; 
        msg.lots[i].p1_y = worldlots[i][3]; 
        msg.lots[i].p2_x = worldlots[i][4];
        msg.lots[i].p2_y = worldlots[i][5];
        msg.lots[i].p3_x = worldlots[i][6];
        msg.lots[i].p3_y = worldlots[i][7];

        LOG4CXX_TRACE(logger_parkinggo, "p0_x = " <<  msg.lots[i].p0_x);
        LOG4CXX_TRACE(logger_parkinggo, "p0_y = " <<  msg.lots[i].p0_y);
        LOG4CXX_TRACE(logger_parkinggo, "p1_x = " <<  msg.lots[i].p1_x);
        LOG4CXX_TRACE(logger_parkinggo, "p1_y = " <<  msg.lots[i].p1_y);
        LOG4CXX_TRACE(logger_parkinggo, "p2_x = " <<  msg.lots[i].p2_x);
        LOG4CXX_TRACE(logger_parkinggo, "p2_y = " <<  msg.lots[i].p2_y);
        LOG4CXX_TRACE(logger_parkinggo, "p3_x = " <<  msg.lots[i].p3_x);
        LOG4CXX_TRACE(logger_parkinggo, "p4_y = " <<  msg.lots[i].p3_y);
    }

    msg.radar1_alarm_level = data->rader1_alarm_level;
    msg.radar2_alarm_level = data->rader2_alarm_level;
    msg.radar3_alarm_level = data->rader3_alarm_level;
    msg.radar4_alarm_level = data->rader4_alarm_level;
    msg.radar5_alarm_level = data->rader5_alarm_level;
    msg.radar6_alarm_level = data->rader6_alarm_level;
    msg.radar7_alarm_level = data->rader7_alarm_level;
    msg.radar8_alarm_level = data->rader8_alarm_level;
    msg.radar9_alarm_level = data->rader9_alarm_level;
    msg.radar10_alarm_level = data->rader10_alarm_level;
    msg.radar11_alarm_level = data->rader11_alarm_level;
    msg.radar12_alarm_level = data->rader12_alarm_level;
    msg.radar13_alarm_level = data->rader13_alarm_level;
    msg.radar14_alarm_level = data->rader14_alarm_level;
    msg.radar15_alarm_level = data->rader15_alarm_level;
    msg.radar16_alarm_level = data->rader16_alarm_level;

    msg.parking_rect_point0_x = data->parking_rect_point0_x;
    msg.parking_rect_point0_y = data->parking_rect_point0_y;
    msg.parking_rect_point1_x = data->parking_rect_point1_x;
    msg.parking_rect_point1_y = data->parking_rect_point1_y;
    msg.parking_rect_point2_x = data->parking_rect_point2_x;
    msg.parking_rect_point2_y = data->parking_rect_point2_y; 
    msg.parking_rect_point3_x = data->parking_rect_point3_x;
    msg.parking_rect_point3_y = data->parking_rect_point3_y; 

    msg.GPS_x = data->GPS_x;
    msg.GPS_y = data->GPS_y;
    msg.GPS_z = data->GPS_z;

    printf("cheng debug: GPS_x = %d \n", msg.GPS_x);
    printf("cheng debug: GPS_y = %d \n", msg.GPS_y);
    printf("cheng debug: GPS_z = %d \n", msg.GPS_z);

    msg.car_parking_status = data->car_paring_status;

    printf("cheng debug: car_parking_status = %d \n", msg.car_parking_status);

    msg.TimeStampex.push_back(data->TimeStampex[0]);
    msg.TimeStampex.push_back(data->TimeStampex[1]);
    msg.TimeStampex.push_back(data->TimeStampex[2]);


    printf("cheng debug: TimeStampex = %d \n", data->TimeStampex[0]);
    printf("cheng debug: TimeStampey = %d \n", data->TimeStampex[1]);
    printf("cheng debug: TimeStampez = %d \n", data->TimeStampex[2]);

    msg.trail_arc_radius = data->trail_arc_radius;
    msg.trail_arc_angle = data->trail_arc_angle;
    msg.parking_mode_select01 = data->parking_mode_select01;
    msg.parking_mode_select02 = data->parking_mode_select02;
    msg.gear_status_actual = data->gear_status_actual;
    msg.gear_status_pre = data->gear_status_pre;
    msg.McuKey = data->McuKey;
    msg.WheelSpeed.push_back(data->WheelSpeed[0]);
    msg.WheelSpeed.push_back(data->WheelSpeed[1]);
    msg.WheelSpeed.push_back(data->WheelSpeed[2]);
    msg.WheelSpeed.push_back(data->WheelSpeed[3]);
    msg.WheelSpeed.push_back(data->WheelSpeed[4]);

    double time0;
    time0 = radar_begin_tm.tv_sec*1000.0 + radar_begin_tm.tv_usec/1000.0 -radar_timedelay;

    msg.radar_begin_sec  = (long int)(time0/1000);
    msg.radar_begin_usec = (long int)((time0 - msg.radar_begin_sec*1000)*1000);

#if __DEBUG_PRINT_TIME_POSI__  
    int radar_posi[3];
    int GPS_posi[3];

    radar_posi[0] = data->TimeStampex[0];
    radar_posi[1] = data->TimeStampex[1];
    radar_posi[2] = data->TimeStampex[2];

    GPS_posi[0] = msg.GPS_x;
    GPS_posi[1] = msg.GPS_y;
    GPS_posi[2] = msg.GPS_z;

    char namestring[20] = "time_radarstart_x";
    timeval radar_begin_tmx;
    radar_begin_tmx.tv_sec = msg.radar_begin_sec;
    radar_begin_tmx.tv_usec = msg.radar_begin_usec;
    printtime(radar_begin_tmx, namestring,radar_posi, GPS_posi);

#endif

    double time1 = radar_end_tm.tv_sec*1000.0 + radar_end_tm.tv_usec/1000.0 -radar_timedelay;

    msg.radar_end_sec  = (long int)(time1/1000);
    msg.radar_end_usec = (long int)((time1 - msg.radar_end_sec*1000)*1000);

    // msg.radar_begin_sec = radar_begin_tm.tv_sec;
    // msg.radar_begin_usec = radar_begin_tm.tv_usec;

    // msg.radar_end_sec = radar_end_tm.tv_sec;
    // msg.radar_end_usec = radar_end_tm.tv_usec;

    pub_mcu.publish(msg);
}

