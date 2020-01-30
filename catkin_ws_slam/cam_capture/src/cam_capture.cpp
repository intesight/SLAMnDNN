#include "parkinggo/parkinggo_common.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <log4cxx/logger.h>
#include <log4cxx/propertyconfigurator.h>

#include "message/groundPoint.h"
#include "message/slamParkingPoint.h"
#include "cam_capture/parkinggo_image.h"
#include "comm_mcu/mcu_msg.h"
#include "comm_mcu/multiple_msg.h"

#include "parkinggo/capture_v4l2.h"
#include "lut_lib/image_view_convert.h"
#include "video_capture.h"

#include <opencv2/videoio.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <opencv2/core/utility.hpp>
#include <opencv2/tracking.hpp>
#include <ros/ros.h>
// #include <conio.h>
#include <pthread.h>
#include <stdio.h>
#include <sys/select.h>
#include <termios.h>
#include <stropts.h>


using namespace cv;
using namespace std;
using namespace log4cxx;

static void get_rawdata(Mat& dst1, Mat& dst2, Mat& dst3, Mat& dst4);
static void publish_image(ros::Publisher& publisher, ros::Time timestamp, Mat& image, int& frame_no, short int* coord);
static void publish_image(image_transport::Publisher& publisher, Mat& image, int& frame_no, short int* coord);

//add
static void publish_dso_data(ros::Publisher& publisher, ros::Time timestamp, Mat& image, int& frame_no, int* coord);
void* save_picture(void* args);
int _kbhit();



static Mat get_stitch_data();

static int find_camera();

static void save_parking_video(VideoWriter& encoder, FILE*& txt, bool& recording, char& status,  int* coord,
                                int* GPScoord,Mat& front, Mat& back, Mat& left, Mat& right,float* ExpoTime);

static void image_radar_synchronization(int radar_sync_posi[3], int GPS_sync_posi[3]);
static void printtime(FILE* fp, timeval radar_begin_tm, char namestring[20], int radar_posi[3], int GPS_posi[3]);
static void printtime_stamp(FILE* fp, double radarmatch_time,double image_time,char namestring[20], int radar_posi[3], int GPS_posi[3]);

#define IMAGE_WIDTH           (1280)
#define IMAGE_HEIGHT          (720)
#define SLAM_IMAGE_WIDTH      (480)
#define SLAM_IMAGE_HEIGHT     (360)

#define DEVICE_NAME1 "Live Streaming USB Device"
#define DEVICE_NAME2 "USB Capture HDMI"

#define SAVE_PARKING_VIDEO
//edit
VideoWriter right_encoder;
#define PUB_DSO_FLAG

#define LOCAL_VIDEO_TEST
#define RADARTIME_COMPENSATION_TEST
#define __DEBUG_PRINT_TIME_LOG__ 1

Mat raw_image_picture(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);

extern unsigned int Lut_Fsv_View[IMAGE_WIDTH * IMAGE_HEIGHT];
extern unsigned int Lut_Front[LUT_POS_FB];
extern unsigned int Lut_Back[LUT_POS_FB];
extern unsigned int Lut_Left[LUT_POS_LR];
extern unsigned int Lut_Right[LUT_POS_LR];
extern unsigned long int Wt_Lut_Front[LUT_WT_FB];
extern unsigned long int Wt_Lut_Back[LUT_WT_FB];
extern unsigned long int Wt_Lut_Left[LUT_WT_LR];
extern unsigned long int Wt_Lut_Right[LUT_WT_LR];
extern unsigned char SVM_BUFFERuyvy[DST_WIDTH * DST_HIGHT * 2];
extern unsigned char SVM_BUFFERuyvy270[DST_WIDTH * DST_HIGHT * 2];
extern unsigned char SVM_BUFFERyuyv[DST_WIDTH * DST_HIGHT * 2];
extern CvPoint1  car_up_left, car_down_right;
extern TabBev **bev_Table[4];
static int front_fov_height, back_fov_height, left_fov_width, right_fov_width;

static Mat front_rawdata(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
static Mat rear_rawdata(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
static Mat left_rawdata(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
static Mat right_rawdata(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);

static unsigned char mem_video_front[IMAGE_WIDTH * IMAGE_HEIGHT * 2];
static unsigned char mem_video_back[IMAGE_WIDTH * IMAGE_HEIGHT * 2];
static unsigned char mem_video_left[IMAGE_WIDTH * IMAGE_HEIGHT * 2];
static unsigned char mem_video_right[IMAGE_WIDTH * IMAGE_HEIGHT * 2];

static unsigned char mem_video_front_mirror[IMAGE_WIDTH * IMAGE_HEIGHT * 2];
static unsigned char mem_video_back_mirror[IMAGE_WIDTH * IMAGE_HEIGHT * 2];
static unsigned char mem_video_left_mirror[IMAGE_WIDTH * IMAGE_HEIGHT * 2];
static unsigned char mem_video_right_mirror[IMAGE_WIDTH * IMAGE_HEIGHT * 2];

static pthread_mutex_t read_data_lock;
static pthread_mutex_t mcu_callback_lock;

static int frame_no = 0;
static char parking_status = 0;
static short int TimeStampex[3] = {0};
static short int GPS_coord[3] = {0};
static int image_delayTime = 0; //100ms image time delay

static struct timeval tm_startImage;
static struct timeval tm_endImage;
static struct timeval tm_startRadar;
static struct timeval tm_endRadar;
static double velocity = 0.0;
static double radar_pre_posi[3] ={0.0};
static FILE* radartxt= NULL;
static FILE* radarmatch= NULL;

static PARKINGGO_CONFIG parkinggo_configs;

static LoggerPtr logger_parkinggo(Logger::getLogger("cam_capture"));

typedef struct
{
    double radar_time;
    short int radar_x;
    short int radar_y;
    short int radar_z;
    short int GPS_x;
    short int GPS_y;
    short int GPS_z;
}radar_data_s;

typedef struct
{
    Mat front;
    Mat back;
    Mat left;
    Mat right;
    short int Coord_XYZ[3] = {0};
    short int GPS_coord[3] = {0};
    struct timeval tm_startImage;
    struct timeval tm_endImage;
    struct timeval tm_startRadar;
    struct timeval tm_endRadar;
    float Expo_Time_SL[8];
}Cam_Data_S;
Cam_Data_S Receive_Data;
struct timeval time_old,time_now;

static vector<radar_data_s> radar_data_v;

vector<timeval> Image_Time;
vector<timeval> Radar_Time;
vector<short int> Position_X;
vector<short int> Position_Y;
vector<Point3i> Coord_XYZ;
#define RESERVED_NUMBER 6
static bool Time_Status = false;
//计算曝光时间
float expo_time[8] = {0.0};
// void cal_expo_time(unsigned char* front, unsigned char* back, unsigned char* left, unsigned char* right,float* Expo_Time);

void mcu_msg_callback(const comm_mcu::mcu_msg& msg)
{
    LOG4CXX_TRACE(logger_parkinggo, "mcu_msg_callback: acquire mcu_callback_lock");
    pthread_mutex_lock(&mcu_callback_lock);

    parking_status = msg.car_parking_status;
    TimeStampex[0] = msg.TimeStampex[0];
    TimeStampex[1] = msg.TimeStampex[1];
    TimeStampex[2] = msg.TimeStampex[2];

    GPS_coord[0] = msg.GPS_x;
    GPS_coord[1] = msg.GPS_y;
    GPS_coord[2] = msg.GPS_z;

    // 读取雷达数据时间
    //tm_startRadar.tv_sec = 1000;
    //tm_startRadar.tv_usec = 2000;

    tm_startRadar.tv_sec = msg.radar_begin_sec;
    tm_startRadar.tv_usec = msg.radar_begin_usec;
    tm_endRadar.tv_sec = msg.radar_end_sec;
    tm_endRadar.tv_usec = msg.radar_end_usec;

    double radartime;
    radartime = tm_startRadar.tv_sec*1000.0 + tm_startRadar.tv_usec/1000.0;

    radar_data_s radar_data;
    radar_data.radar_time = radartime;
    radar_data.radar_x = msg.TimeStampex[0];
    radar_data.radar_y = msg.TimeStampex[1];
    radar_data.radar_z = msg.TimeStampex[2];
    radar_data.GPS_x = msg.GPS_x;
    radar_data.GPS_y = msg.GPS_y;
    radar_data.GPS_z = msg.GPS_z;

    radar_data_v.push_back(radar_data);

#if __DEBUG_PRINT_TIME_LOG__
    int radardata_tmp[3];
    int GPS_tmp[3];
    radardata_tmp[0] = radar_data.radar_x;
    radardata_tmp[1] = radar_data.radar_y;
    radardata_tmp[2] = radar_data.radar_z;
    GPS_tmp[0] = radar_data.GPS_x;
    GPS_tmp[1] = radar_data.GPS_y;
    GPS_tmp[2] = radar_data.GPS_z;

    timeval radar_begin_tm; 
    radar_begin_tm.tv_sec = tm_startRadar.tv_sec;
    radar_begin_tm.tv_usec = tm_startRadar.tv_usec;

    char namestring[20] = "radardata_recieve";
    printtime(radartxt, radar_begin_tm, namestring, radardata_tmp, GPS_tmp);

#endif

    pthread_mutex_unlock(&mcu_callback_lock);
    LOG4CXX_TRACE(logger_parkinggo, "mcu_msg_callback: release mcu_callback_lock");
}
void multiple_msg_callback(const comm_mcu::multiple_msgConstPtr& msg) 
{
    cout<<"receive data"<<endl;
    gettimeofday(&time_now, NULL);
    double once_time = (double)getTickCount();
    //更新 Cam_Data_S 结构体数据
    try{
        //图像、Coord_XYZ数据更新
        // Receive_Data.front  = cv_bridge::toCvCopy(msg->image_front, "8UC2")->image;
        // Receive_Data.back  = cv_bridge::toCvCopy(msg->image_back, "8UC2")->image;
        // Receive_Data.left  = cv_bridge::toCvCopy(msg->image_left, "8UC2")->image;
        // Receive_Data.right  = cv_bridge::toCvCopy(msg->image_right, "8UC2")->image;
        Receive_Data.Coord_XYZ[0] = msg->coord_X;
        Receive_Data.Coord_XYZ[1] = msg->coord_Y;
        Receive_Data.Coord_XYZ[2] = msg->coord_Z;
        //曝光时间
        Receive_Data.Expo_Time_SL[0] = msg->expo_front_s;
        Receive_Data.Expo_Time_SL[1] = msg->expo_front_l;
        Receive_Data.Expo_Time_SL[2] = msg->expo_back_s;
        Receive_Data.Expo_Time_SL[3] = msg->expo_back_l;
        Receive_Data.Expo_Time_SL[4] = msg->expo_left_s;
        Receive_Data.Expo_Time_SL[5] = msg->expo_left_l;
        Receive_Data.Expo_Time_SL[6] = msg->expo_right_s;
        Receive_Data.Expo_Time_SL[7] = msg->expo_right_l;

        float time_measure = (time_now.tv_sec - time_old.tv_sec)*1000.0 + (time_now.tv_usec - time_old.tv_usec)/1000.0;
        cout<<"timeval  当前响应时间与上次响应时间之差："<<time_measure<<"ms"<<endl;
        once_time = (double)getTickCount() - once_time;
        printf("run once time = %f\n",once_time*1000.0/getTickFrequency());
        printf("前相机曝光时间：%f,%f\n",Receive_Data.Expo_Time_SL[0],Receive_Data.Expo_Time_SL[1]);
        printf("后相机曝光时间：%f,%f\n",Receive_Data.Expo_Time_SL[2],Receive_Data.Expo_Time_SL[3]);
        printf("左相机曝光时间：%f,%f\n",Receive_Data.Expo_Time_SL[4],Receive_Data.Expo_Time_SL[5]);
        printf("右相机曝光时间：%f,%f\n",Receive_Data.Expo_Time_SL[6],Receive_Data.Expo_Time_SL[7]);
        memcpy(&time_old,&time_now,sizeof(timeval));
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Update 'Receive_Data' failed.");
    }
}

int main(int argc, char** argv)
{
    // 定义线程的 id 变量，多个变量使用数组
    int NUM_THREADS = 1;
    pthread_t tids[NUM_THREADS];
    for(int i = 0; i < NUM_THREADS; ++i)
    {
        //参数依次是：创建的线程id，线程参数，调用的函数，传入的函数参数
        int ret = pthread_create(&tids[i], NULL, save_picture, NULL);
        if (ret != 0)
        {
           cout << "pthread_create error: error_code=" << ret << endl;
        }
    }


    set_cpu(logger_parkinggo, CPU_ID_CAM_CAPTURE);

    load_config(logger_parkinggo, parkinggo_configs);

    LOG4CXX_INFO(logger_parkinggo, "Camera Capture start!");

    pthread_t tStitchImage;
    
    pthread_mutex_init(&read_data_lock, NULL);
    pthread_mutex_init(&mcu_callback_lock, NULL);

    // camera capture ros module
    ros::init(argc, argv, "cam_capture");
    ros::NodeHandle nh;

    //发布原始图像数据信息给DSO
    ros::NodeHandle it_dso;
    ros::Publisher pub_raw_data = it_dso.advertise<cam_capture::parkinggo_image>("raw_image_radar", 1);

     ros::Subscriber sub = nh.subscribe("mcu_msg", 2, mcu_msg_callback);

    //订阅multiple_msg消息
    ros::Subscriber multiple_sub = nh.subscribe("multi/image", 2, multiple_msg_callback);

    // Old type images for SLAM
    image_transport::ImageTransport it_raw_front(nh);
    image_transport::Publisher pub_raw_front = it_raw_front.advertise("raw_front", 1);

    image_transport::ImageTransport it_raw_rear(nh);
    image_transport::Publisher pub_raw_rear = it_raw_rear.advertise("raw_rear", 1);

    image_transport::ImageTransport it_raw_left(nh);
    image_transport::Publisher pub_raw_left = it_raw_left.advertise("raw_left", 1);

    image_transport::ImageTransport it_raw_right(nh);
    image_transport::Publisher pub_raw_right = it_raw_right.advertise("raw_right", 3);

    // New type images for DNN
    ros::NodeHandle it_front;
    ros::Publisher pub_front = it_front.advertise<cam_capture::parkinggo_image>("front_image", 1);

    ros::NodeHandle it_rear;
    ros::Publisher pub_rear = it_rear.advertise<cam_capture::parkinggo_image>("back_image", 1);

    ros::NodeHandle it_left;
    ros::Publisher pub_left = it_left.advertise<cam_capture::parkinggo_image>("left_image", 1);

    ros::NodeHandle it_right;
    ros::Publisher pub_right = it_right.advertise<cam_capture::parkinggo_image>("right_image", 3);

    Mat dst1(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
    Mat dst2(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
    Mat dst3(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
    Mat dst4(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);

    cv::Mat raw_image_front(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
    cv::Mat raw_image_rear(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
    cv::Mat raw_image_left(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
    cv::Mat raw_image_right(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);

    cv::Mat slam_image_front(SLAM_IMAGE_WIDTH, SLAM_IMAGE_HEIGHT, CV_8UC3);
    cv::Mat slam_image_rear(SLAM_IMAGE_WIDTH, SLAM_IMAGE_HEIGHT, CV_8UC3);
    cv::Mat slam_image_left(SLAM_IMAGE_WIDTH, SLAM_IMAGE_HEIGHT, CV_8UC3);
    cv::Mat slam_image_right(SLAM_IMAGE_WIDTH, SLAM_IMAGE_HEIGHT, CV_8UC3);

    short int coord[3];
    short int GPSCoord[3];
    char status;
    printf("continue is ok!\n");

#if __DEBUG_PRINT_TIME_LOG__
    radartxt = fopen("radardata_recive.txt", "w");
    fprintf(radartxt, "time, radar_x, radar_y, radar_z, GPS_x, GPS_y, GPS_z \n");

    radarmatch = fopen("radarmatch.txt", "w");
    fprintf(radarmatch, "time, radar_x, radar_y, radar_z, GPS_x, GPS_y, GPS_z \n");

#endif

    // if (parkinggo_configs.device == S32V)
    //if (0)
    if (parkinggo_configs.device == S32V)
    {
        // find attached S32V input
        char dev[20];
        int dev_id = find_camera();
        if (dev_id == -1)
        {
            LOG4CXX_ERROR(logger_parkinggo, "Can't find S32V camera!");
        }

        snprintf(dev, 20, "/dev/video%d", dev_id);

        load_config_file(parkinggo_configs.svm_dir.c_str(), car_up_left, car_down_right, front_fov_height, back_fov_height, left_fov_width, right_fov_width);
        if (dev_id != -1)
        {
            init_video_capture(dev);
        }
        
        ReadOrigialmage();
        ReadLut();
    }

    cv::VideoCapture* cap;
    // if (parkinggo_configs.device == VIDEO)
    //if (1)
    if (parkinggo_configs.device == VIDEO)
    {
        cap = new cv::VideoCapture(parkinggo_configs.input_video.c_str());
    }

    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setschedpolicy(&attr, SCHED_RR);

    int pthread_stitch_flag = 0;

#ifdef SAVE_PARKING_VIDEO
    VideoWriter encoder;
    FILE* txt = NULL;
    bool recording = false;

    char path[255];
    sprintf(path, "mkdir -p %s\n", (parkinggo_configs.base_dir + "videos").c_str());
    system(path);    
#endif
    //string videoFile = "/home/chengguoqiang/cheng/video_data/slam_video_11_14/20181114-142833.avi";
    //cv::VideoCapture cap1(videoFile); 

    ros::Rate loop_rate(33);

    while (ros::ok())
    {
        double t = (double)cvGetTickCount();
     
        LOG4CXX_TRACE(logger_parkinggo, "main pthread: start >>>>>>>>>>>>>>>>>>>>>>>>>>");      
        LOG4CXX_TRACE(logger_parkinggo, "main pthread: run once time start");

        LOG4CXX_TRACE(logger_parkinggo, "main pthread: acquire read_data_lock");
        pthread_mutex_lock(&read_data_lock);

        Mat stitch_image;
        if (parkinggo_configs.device == VIDEO)
        {
	    double t1 = (double)cvGetTickCount();
            //读取视频开始时间
            gettimeofday(&tm_startImage, NULL);
            #if 1
            *cap >> stitch_image;
            if(stitch_image.empty())
            {
                cap = new VideoCapture(parkinggo_configs.input_video.c_str());
                *cap >> stitch_image;
            }
            
            Mat image_front = stitch_image(Rect(0, 0, IMAGE_WIDTH, IMAGE_HEIGHT));
            Mat image_left = stitch_image(Rect(IMAGE_WIDTH, 0, IMAGE_WIDTH, IMAGE_HEIGHT));
            Mat image_rear = stitch_image(Rect(0, IMAGE_HEIGHT, IMAGE_WIDTH, IMAGE_HEIGHT));
            Mat image_right = stitch_image(Rect(IMAGE_WIDTH, IMAGE_HEIGHT, IMAGE_WIDTH, IMAGE_HEIGHT));
            image_front.copyTo(front_rawdata);
            image_rear.copyTo(rear_rawdata);
            image_left.copyTo(left_rawdata);
            image_right.copyTo(right_rawdata);
            #endif
            //读取视频结束时间
            gettimeofday(&tm_endImage, NULL);
            //imshow("stitch_image", stitch_image);
            double t2 = (double)cvGetTickCount();
	    printf("cheng debug:xxxxxxxxxxxxxxxxxxx Video time =  %f ms \n",(t2-t1)/cvGetTickFrequency()/1000);
        }
        else
        {
            try 
            {
                double t1 = (double)cvGetTickCount();
               
                // 读取视频开始时间
                gettimeofday(&tm_startImage, NULL);

                char namestring[20] = "startImage";
              //  printtime(tm_startImage, namestring);

                // =================== 读取视频 ===================
             	get_rawdata(dst1, dst2, dst3, dst4);

                //cap1>>front_rawdata;  

                // 读取视频结束时间
                gettimeofday(&tm_endImage, NULL);

               double t2 = (double)cvGetTickCount();

                //  Rect front_roi(0,0,1280,720);
    
                // front_rawdata = front_rawdata(front_roi);
                // right_rawdata = left_rawdata = rear_rawdata = front_rawdata;

               printf("cheng debug:xxxxxxxxxxxxxxxxxxx Capture time =  %f ms \n",(t2-t1)/cvGetTickFrequency()/1000);
            }
            catch(...)
            {
                LOG4CXX_ERROR(logger_parkinggo, "main pthread: Exception catched");
            }
        }
	//计算曝光时间
        // cal_expo_time(mem_video_front, mem_video_back, mem_video_left, mem_video_right,expo_time);

        pthread_mutex_unlock(&read_data_lock);
        LOG4CXX_TRACE(logger_parkinggo, "main pthread: release read_data_lock");
        
        LOG4CXX_TRACE(logger_parkinggo, "main pthread: get_rawdata finish");
        LOG4CXX_TRACE(logger_parkinggo, "main pthread: get_rawdata time = " << ((double)cvGetTickCount() - t)/(cvGetTickFrequency()*1000) << " ms");

        front_rawdata.copyTo(raw_image_front);  
        rear_rawdata.copyTo(raw_image_rear);  
        left_rawdata.copyTo(raw_image_left);  
        right_rawdata.copyTo(raw_image_right);

        rear_rawdata.copyTo(raw_image_picture);  
        

        // imshow("rawdata", right_rawdata);

        LOG4CXX_TRACE(logger_parkinggo, "main pthread: acquire mcu_callback_lock");

        loop_rate.sleep();
        ros::spinOnce();

        pthread_mutex_lock(&mcu_callback_lock);

        coord[0] = TimeStampex[0];
        coord[1] = TimeStampex[1];
        coord[2] = TimeStampex[2];

        GPSCoord[0] = GPS_coord[0];
        GPSCoord[1] = GPS_coord[1];
        GPSCoord[2] = GPS_coord[2];

        status = parking_status;

        pthread_mutex_unlock(&mcu_callback_lock);
        LOG4CXX_TRACE(logger_parkinggo, "main pthread: release mcu_callback_lock");

        int radar_sync_posi[3];
        int GPS_sync_posi[3];
        image_radar_synchronization(radar_sync_posi, GPS_sync_posi);

#ifdef  SAVE_PARKING_VIDEO
        static int record = 0;
        if (status == 1 || record == 1)
        {
            record = 1;
            status = 1;
        }

        status = 1;

     //   imshow("raw_image_right", raw_image_right);
        printf("cheng debug:status = %d \n", status);

        double txx = (double)cvGetTickCount();

        // imshow("raw_image_front" , raw_image_front);
        // imshow("raw_image_rear" , raw_image_rear);
        // imshow("raw_image_left" , raw_image_left);
        // imshow("raw_image_right" , raw_image_right);

    //    save_parking_video(encoder, txt, recording, status, coord, GPSCoord, raw_image_front, raw_image_rear, raw_image_left, raw_image_right);

#ifdef PUB_DSO_FLAG
//static void save_parking_video(VideoWriter& encoder, FILE*& txt, bool& recording, char& status,  int* coord,int* GPScoord,Mat& front, Mat& back, Mat& left, Mat& right,float* ExpoTime);
        ros::Time time_now = ros::Time::now();
        publish_dso_data(pub_raw_data, time_now, raw_image_rear, frame_no, radar_sync_posi);
        // imshow("111",raw_image_rear);
        waitKey(10);
        //cam_capture::parkinggo_image msg;
#else
        save_parking_video(encoder, txt, recording, status, radar_sync_posi, GPS_sync_posi, raw_image_front, raw_image_rear, raw_image_left, raw_image_right,Receive_Data.Expo_Time_SL);
#endif

        txx = (double)cvGetTickCount() - txx;

        printf("cheng debug: save_parking_video time = %f ms\n" , txx/(cvGetTickFrequency()*1000));

#else

        double fx = 0.0;
        double fy = 0.0;
        // resize(front_rawdata, slam_image_front, Size(SLAM_IMAGE_WIDTH, SLAM_IMAGE_HEIGHT), fx, fy, CV_INTER_CUBIC); 
        // resize(rear_rawdata, slam_image_rear, Size(SLAM_IMAGE_WIDTH, SLAM_IMAGE_HEIGHT), fx, fy, CV_INTER_CUBIC); 
        // resize(left_rawdata, slam_image_left, Size(SLAM_IMAGE_WIDTH, SLAM_IMAGE_HEIGHT), fx, fy, CV_INTER_CUBIC); 
        // resize(right_rawdata, slam_image_right, Size(SLAM_IMAGE_WIDTH, SLAM_IMAGE_HEIGHT), fx, fy, CV_INTER_CUBIC); 
      
        // publish_image(pub_raw_front, slam_image_front, frame_no, coord);
        // publish_image(pub_raw_rear, slam_image_rear, frame_no, coord);
        // publish_image(pub_raw_left, slam_image_left, frame_no, coord);
        // publish_image(pub_raw_right, slam_image_right, frame_no, coord);
      
        ros::Time timestamp = ros::Time::now();
        // publish_image(pub_front, timestamp, raw_image_front, frame_no, coord);
        // publish_image(pub_rear, timestamp, raw_image_rear, frame_no, coord);
        // publish_image(pub_left, timestamp, raw_image_left, frame_no, coord);
        publish_image(pub_right, timestamp, raw_image_right, frame_no, coord);

#endif

        t = (double)cvGetTickCount() - t;
        LOG4CXX_TRACE(logger_parkinggo, "main pthread: run once time = " << t/(cvGetTickFrequency()*1000) << " ms");
        LOG4CXX_TRACE(logger_parkinggo, "main pthread: run once time finish");
        LOG4CXX_TRACE(logger_parkinggo, "main pthread: end <<<<<<<<<<<<<<<<<<<<<<<<<<<");

        printf("cheng debug: cam_capture pthread: run once time = %f ms\n" , t/(cvGetTickFrequency()*1000));

        if (cv::waitKey(10) >=0) // 'ESC'
        {
           break;
        }

        // loop_rate.sleep();
        // ros::spinOnce();
    }

    LOG4CXX_INFO(logger_parkinggo, "Camera Capture finish!");

    return 0;
}

void image_radar_synchronization(int radar_sync_posi[3], int GPS_sync_posi[3])
{
    int rtn = 0;

    double image_time = tm_startImage.tv_sec*1000.0 + tm_startImage.tv_usec/1000.0 - image_delayTime;
    static int pre_radar_size;

    int index_min = 0;
    int radar_size = radar_data_v.size();

    int radar_increament = radar_size - pre_radar_size;

    printf("radar_size = %d radar_increament = %d \n", radar_size, radar_increament);

    if(radar_size == 0)
        return;

    double time_offset_min = fabs(radar_data_v[0].radar_time - image_time);//初始时刻为最小匹配时间差
    double time_offset_tmp = 0;

    //遍历所有的vector,找出离imagetime最近的radartime，并记录其位置
    for(int i = 1; i < radar_size; i++)
    {
        radar_data_s radar_tmp = radar_data_v[i];
        time_offset_tmp = fabs(radar_tmp.radar_time - image_time);

        if (time_offset_min >  time_offset_tmp)
        {
            time_offset_min = time_offset_tmp;
            index_min = i;
        }
    }

    time_offset_tmp = radar_data_v[index_min].radar_time - image_time;

    printf("index_min = %d time_offset_tmp = %f \n", index_min, time_offset_tmp);

    double delta_time = radar_data_v[index_min].radar_time - radar_pre_posi[2];

    printf("radar_pre_posi[0] = %f radar_pre_posi[1] = %f radar_pre_posi[2] = %f  \n", radar_pre_posi[0], radar_pre_posi[1], radar_pre_posi[2]);

    printf("radar_time = %f delta_time = %f \n",radar_data_v[index_min].radar_time ,delta_time);

    if(fabs(delta_time) > 1e-6)
    {
        velocity = sqrt((radar_data_v[index_min].radar_x - radar_pre_posi[0]) * (radar_data_v[index_min].radar_x - radar_pre_posi[0])
            + (radar_data_v[index_min].radar_y - radar_pre_posi[1]) * (radar_data_v[index_min].radar_y - radar_pre_posi[1]) ) / fabs(delta_time); //计算前一时刻的radar速度
    }
    else//当系统开始时,由于图像延时100ms，radar的时间会大于图像的时间,导致前几次选取的radar数据相同（第一次的radar数据与前几次的图像数据最接近）,所以delta_time为0
    {
        if( radar_size > 1) //当保留的不止一个radar数据时,用最新时刻的数据计算速度
        {
            delta_time = radar_data_v[radar_size-1].radar_time - radar_data_v[radar_size-2].radar_time;
            
            printf("new delta_time = %f \n", delta_time);

            velocity = sqrt((radar_data_v[radar_size-1].radar_x - radar_data_v[radar_size-2].radar_x) * (radar_data_v[radar_size-1].radar_x - radar_data_v[radar_size-2].radar_x)
                          + (radar_data_v[radar_size-1].radar_y - radar_data_v[radar_size-2].radar_y) * (radar_data_v[radar_size-1].radar_y - radar_data_v[radar_size-2].radar_y))/fabs(delta_time);
        }
        else
            velocity = 0.0;
    }
        
    printf("velocity = %f \n", velocity);

    if(time_offset_tmp > 0) //被选择radar时间大于当前图像时间
    {
        radar_sync_posi[0] = radar_data_v[index_min].radar_x - velocity * time_offset_tmp; //s = s1 - v*t
        radar_sync_posi[1] = radar_data_v[index_min].radar_y - velocity * time_offset_tmp;
        radar_sync_posi[2] = radar_data_v[index_min].radar_z - velocity * time_offset_tmp;

        GPS_sync_posi[0] = radar_data_v[index_min].GPS_x - velocity * time_offset_tmp;
        GPS_sync_posi[1] = radar_data_v[index_min].GPS_y - velocity * time_offset_tmp;
        GPS_sync_posi[2] = radar_data_v[index_min].GPS_z - velocity * time_offset_tmp;

                //保留当前时刻的位置和时间
        radar_pre_posi[0] = radar_data_v[index_min].radar_x;
        radar_pre_posi[1] = radar_data_v[index_min].radar_y;
        radar_pre_posi[2] = radar_data_v[index_min].radar_time;

        double radarmatch_time = radar_data_v[index_min].radar_time;
        char namestring[20] = "startImage_delay";
        printtime_stamp(radarmatch, radarmatch_time, image_time, namestring, radar_sync_posi, GPS_sync_posi);

        if(index_min >  0)  //非第一个radar数据,需要删掉vector中的数据
        {
            if(index_min > 2) //刚开始时,图像数据延迟100ms,导致前几个的radar时间会大于当前图像的时间,所以刚开始时的数据不应该马上就删掉，应该保留，用于匹配接下来的图像时间
            {
                if(radar_increament > 1)
                {
                    radar_data_v.erase(radar_data_v.begin());
                    radar_data_v.erase(radar_data_v.begin());
                }
                else
                    radar_data_v.erase(radar_data_v.begin());
            }         
        }

    }
    else//被选择radar时间小于于当前图像时间
    {

        radar_sync_posi[0] = radar_data_v[index_min].radar_x + velocity * fabs(time_offset_tmp);
        radar_sync_posi[1] = radar_data_v[index_min].radar_y + velocity * fabs(time_offset_tmp);
        radar_sync_posi[2] = radar_data_v[index_min].radar_z + velocity * fabs(time_offset_tmp);

        GPS_sync_posi[0] = radar_data_v[index_min].GPS_x + velocity * fabs(time_offset_tmp);
        GPS_sync_posi[1] = radar_data_v[index_min].GPS_y + velocity * fabs(time_offset_tmp);
        GPS_sync_posi[2] = radar_data_v[index_min].GPS_z + velocity * fabs(time_offset_tmp);

        //保留当前时刻的位置和时间
        radar_pre_posi[0] = radar_data_v[index_min].radar_x;
        radar_pre_posi[1] = radar_data_v[index_min].radar_y;
        radar_pre_posi[2] = radar_data_v[index_min].radar_time;

        double radarmatch_time = radar_data_v[index_min].radar_time;
        char namestring[20] = "startImage_delay";
        printtime_stamp(radarmatch, radarmatch_time, image_time, namestring, radar_sync_posi, GPS_sync_posi);

        if(index_min > 2)
        {
            if(radar_increament > 1)
            {
                radar_data_v.erase(radar_data_v.begin());
                radar_data_v.erase(radar_data_v.begin());
            }
            else
                radar_data_v.erase(radar_data_v.begin());
        }
        
    }

    radar_size = radar_data_v.size();

    pre_radar_size = radar_data_v.size();
    printf("radar_sizexx = %d \n", radar_size);

    return ;
}

void publish_image(image_transport::Publisher& publisher, Mat& image, int& frame_no, short int* coord)
{
    std_msgs::Header header = std_msgs::Header();
    header.stamp = ros::Time::now();
    header.seq = frame_no;
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
    publisher.publish(msg);
}

void publish_image(ros::Publisher& publisher, ros::Time timestamp, Mat& image, int& frame_no, short int* coord)
{
    cam_capture::parkinggo_image msg;

    std_msgs::Header header = std_msgs::Header();
    header.stamp = timestamp;
    header.seq = frame_no;
    msg.header = header;

    msg.image = *cv_bridge::CvImage(header, "bgr8", image).toImageMsg();

    msg.coordX = coord[0];
    msg.coordY = coord[1];
    msg.coordZ = coord[2];

    msg.image_begin_sec = tm_startImage.tv_sec;
    msg.image_begin_usec = tm_startImage.tv_usec;
    msg.image_end_sec = tm_endImage.tv_sec;
    msg.image_end_usec = tm_endImage.tv_usec;

    publisher.publish(msg);
}

static void get_rawdata(Mat& dst1, Mat& dst2, Mat& dst3, Mat& dst4)
{
    struct v4l2_format fmt;
    struct buffer *buffers;

    video_capture_streams(mem_video_front, mem_video_back, mem_video_left, mem_video_right);

    Mat src1(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC2, (uchar*)mem_video_front);
    cvtColor(src1, front_rawdata, CV_YUV2BGR_YUY2);

    Mat src2(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC2, (uchar*)mem_video_back);
    cvtColor(src2, rear_rawdata, CV_YUV2BGR_YUY2);

    Mat src3(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC2, (uchar*)mem_video_left);
    cvtColor(src3, left_rawdata, CV_YUV2BGR_YUY2);

    Mat src4(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC2, (uchar*)mem_video_right);
    cvtColor(src4, right_rawdata, CV_YUV2BGR_YUY2);
}

static Mat get_stitch_data()
{
     Mat dst5(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
     struct v4l2_format fmt;

     bev_process(
        SVM_BUFFERuyvy,//result_image_uyvy,
        mem_video_front_mirror,//mem_video_frontyuyv,//front_puyvy,//front_image_uyvy,
        mem_video_back_mirror,// mem_video_backyuyv,//back_puyvy,//back_image_uyvy,
        mem_video_left_mirror,//mem_video_leftyuyv,//left_image_uyvy,
        mem_video_right_mirror,// mem_video_rightyuyv,//right_image_uyvy,
        car_up_left,
        car_down_right,
        front_fov_height,
        right_fov_width,
        DST_WIDTH,//Width,
        IMAGE_WIDTH,//SINGLE_VIEW_WIDTH,
        bev_Table);

    rotate270(SVM_BUFFERuyvy, SVM_BUFFERuyvy270, 804, 1248);
    uyvy_to_yuyv(SVM_BUFFERyuyv, SVM_BUFFERuyvy270, 804, 1248);

    fmt.fmt.pix.height = 804;
    fmt.fmt.pix.width = 1248;

    Mat src5(fmt.fmt.pix.height, fmt.fmt.pix.width, CV_8UC2, (uchar *)SVM_BUFFERyuyv);
    cvtColor(src5, dst5, CV_YUV2BGR_YUY2);

    return dst5;
}


static int find_camera()
{
    int fd;
    char dev[20];
    struct v4l2_capability vcap;
    
    for (int i=0; i<10; i++)
    {
        snprintf(dev, 20, "/dev/video%d", i);
        if ((fd = open(dev, O_RDONLY)) < 0)
            continue;

        if (ioctl(fd, VIDIOC_QUERYCAP, &vcap))
            continue;

        close(fd);

        LOG4CXX_INFO(logger_parkinggo, "/dev/video" << i << ": " << vcap.card);

        if (!strcmp((const char *) vcap.card, DEVICE_NAME2))
            continue;

        return i;
    }

    return -1;
}


static void save_parking_video(VideoWriter& encoder, FILE*& txt, bool& recording, char& status,  int* coord, int* GPScoord,Mat& front, Mat& back, Mat& left, Mat& right,float* ExpoTime)
{
    static FILE *fp_expo;
    if (status == 1 || status == 2 || status == 3)
    {
        // generate filename
        
        struct timeval tmnow;
        struct tm *tm;
        char time_buffer[30], usec_buffer[6];
        char filename_video_front[255], filename_video_right[255],filename_txt[255];
        char file_expotime[255];
       // gettimeofday(&tmnow, NULL);
    //    imshow("front",front);
       imshow("back",back);
    //    imshow("left",left);
    //    imshow("right",right);

       
       tm = localtime(&tm_startImage.tv_sec - image_delayTime/1000);
        strftime(time_buffer, 30, "%Y%m%d-%H%M%S", tm);
        // std::cout << "time_buffer" << time_buffer << std::endl;

        // Zero padding millisec
        std::ostringstream ss;
        ss << std::setw(3) << std::setfill('0') << (int) tm_startImage.tv_usec / 1000;
        sprintf(usec_buffer, "%s", ss.str().c_str());

        if (status == 1 && !recording)
        {
            // Enter parking state, start recording  right_encoder
            sprintf(filename_video_front, "%s/%s-front.avi", (parkinggo_configs.base_dir + "videos").c_str(), time_buffer);
            sprintf(filename_video_right, "%s/%s-right.avi", (parkinggo_configs.base_dir + "videos").c_str(), time_buffer);
            sprintf(filename_txt, "%s/%s.txt", (parkinggo_configs.base_dir + "videos").c_str(), time_buffer);

            sprintf(file_expotime, "%s/%s-expo_time.txt", (parkinggo_configs.base_dir + "videos").c_str(), time_buffer);
        //    encoder.open(filename_video_front, VideoWriter::fourcc('P', 'I', 'M', '1'), 30.0, Size(IMAGE_WIDTH, IMAGE_HEIGHT), false);
            encoder.open(filename_video_front, VideoWriter::fourcc('D', 'I', 'V', 'X'), 20.0, Size(IMAGE_WIDTH, IMAGE_HEIGHT), true);
           right_encoder.open(filename_video_right, VideoWriter::fourcc('P', 'I', 'M', '1'), 20.0, Size(IMAGE_WIDTH, IMAGE_HEIGHT), false);
            // encoder.open(filename_video, VideoWriter::fourcc('F', 'L', 'V', '1'), 30.0, Size(IMAGE_WIDTH, IMAGE_HEIGHT), false);
           //  encoder.open(filename_video, VideoWriter::fourcc('M', 'J', 'P', 'G'), 30.0, Size(IMAGE_WIDTH * 2, IMAGE_HEIGHT * 2), true);
            txt = fopen(filename_txt, "w"); 
            fp_expo = fopen(file_expotime, "w"); 
            fprintf(txt, "time, status, coord_x,coord_y,coord_z, GPS_x, GPS_y, GPS_z \n");
            fprintf(fp_expo, "expo time S L:front,back,left,right\n");
            recording = true;
        }

        if (!recording)
            return;

#if 0
        // Merge images
        Mat merged(IMAGE_HEIGHT * 2, IMAGE_WIDTH * 2, CV_8UC3);
        Mat image_front = merged(Rect(0, 0, IMAGE_WIDTH, IMAGE_HEIGHT));
        Mat image_left = merged(Rect(IMAGE_WIDTH, 0, IMAGE_WIDTH, IMAGE_HEIGHT));
        Mat image_back = merged(Rect(0, IMAGE_HEIGHT, IMAGE_WIDTH, IMAGE_HEIGHT));
        Mat image_right = merged(Rect(IMAGE_WIDTH, IMAGE_HEIGHT, IMAGE_WIDTH, IMAGE_HEIGHT));

        front.copyTo(image_front);
        back.copyTo(image_back);
        left.copyTo(image_left);
        right.copyTo(image_right);

        // encoder one frame
        encoder << merged;
#else
        Mat gray_front(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1);
        Mat gray_right(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1);
    //    cv::cvtColor(front, gray_front,CV_BGR2GRAY);
    //    cv::cvtColor(left, gray_right,CV_BGR2GRAY);
        // 读取视频数据
      //  cv::imshow("gray_front", gray_front);

      double txx = (double)cvGetTickCount();

       encoder << back
       
       ;
    //    right_encoder << gray_right;

       txx = (double)cvGetTickCount() - txx;

        printf("cheng debug: encoder time = %f ms\n" , txx/(cvGetTickFrequency()*1000));

#endif

        fprintf(txt, "%s:%s %d %d %d %d %d %d %d \n ", time_buffer, usec_buffer,status, coord[0], coord[1], coord[2], GPScoord[0], GPScoord[1], GPScoord[2]);
        fprintf(fp_expo, "%f,%f,%f,%f,%f,%f,%f,%f \n ",ExpoTime[0],ExpoTime[1],ExpoTime[2],ExpoTime[3],ExpoTime[4],ExpoTime[5],ExpoTime[6],ExpoTime[7]);
    }
    else if ((status == 4 || status == 5) && recording)
    {
        cout<<"4 || status == 5 test filename tss"<<endl;
        // Quit parking state, stop recording
        encoder.release();
        right_encoder.release();
        if (txt != NULL)
        {
            fclose(txt);
            fclose(fp_expo);
            txt = NULL;
            fp_expo = NULL;
        }
        recording = false;
    }
}


static void publish_dso_data(ros::Publisher& publisher, ros::Time timestamp, Mat& image, int& frame_no, int* coord)
{
    cam_capture::parkinggo_image msg;
    std_msgs::Header header = std_msgs::Header();
    header.stamp = timestamp;
    header.seq = frame_no;
    msg.header = header;

    msg.image = *cv_bridge::CvImage(header, "bgr8", image).toImageMsg();

    imshow("test",image);

    //图像车辆坐标系,      x正方向，由车辆左到右, y正方向，由车辆后到前，坐标原点车辆后轴中心
    //超声波雷达车辆坐标系，x正方向, 由车辆后到前，y正方向，由车辆右到左，坐标原点车辆后轴中心
    msg.coordX = -coord[1];
    msg.coordY = coord[0];
    msg.coordZ = coord[2];

    std::cout << "debug: time stamp " << header.stamp << std::endl;
    printf("coordX = %d \n", msg.coordX );
    printf("coordY = %d \n", msg.coordY);
    printf("angle = %d \n",  msg.coordZ);

    publisher.publish(msg);
}

static void printtime(FILE* fp, timeval radar_begin_tm, char namestring[20], int radar_posi[3], int GPS_posi[3])
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

    fprintf(fp,"%s:%s, %d, %d, %d, %d, %d, %d \n",time_StartRadar, usec_StartRadar, radar_posi[0], radar_posi[1], radar_posi[2], GPS_posi[0], GPS_posi[1],GPS_posi[2]);
}

static void printtime_stamp(FILE* fp, double radarmatch_time, double image_time,char namestring[20], int radar_posi[3], int GPS_posi[3])
{
    timeval time_val;

    time_val.tv_sec = (long int)(image_time/1000);
    time_val.tv_usec =  (long int)((image_time - time_val.tv_sec*1000)*1000);

    struct tm *tmStartRadar;
    timeval radarmatch_val;
    radarmatch_val.tv_sec = (long int)(radarmatch_time/1000);
    radarmatch_val.tv_usec = (long int)((radarmatch_time - radarmatch_val.tv_sec*1000)*1000);

    char time_StartRadar[30], usec_StartRadar[6];
    tmStartRadar = localtime(&radarmatch_val.tv_sec);
    strftime(time_StartRadar, 30, "%Y%m%d-%H%M%S",tmStartRadar);

    std::ostringstream ss;
    ss << std::setw(3) << std::setfill('0') << (int) radarmatch_val.tv_usec / 1000;
    sprintf(usec_StartRadar, "%s", ss.str().c_str());

    fprintf(fp,"radar time %s:%s--image time ", time_StartRadar, usec_StartRadar);

    printtime(fp, time_val, namestring, radar_posi, GPS_posi);
}
void* save_picture(void* args)
{
    static bool save_picture =false;
    char keys[10];
    char s = 's';
    static int count = 0;
    while(1)
    {
        char name_picturre[255];
        sprintf(name_picturre,"%d.jpg",count);

        ostringstream convert;   // stream used for the conversion
        convert << count;
        Mat im(233,400,CV_8UC3,Scalar(0,0,0));
        int value = 0;
        cin>>value;
        if(2 == value)
        {
            printf("the value key is got %d\n",value);
            if(raw_image_picture.empty())
                printf("picture is empty\n");
            else
            {
                printf("name : %s\n",name_picturre);
                // imwrite("name_picturre",im);
                imwrite(name_picturre,raw_image_picture);
                count++;
            }
                
        }
        // if(_kbhit())
        // {
        //     printf("key is %d\n",_kbhit());
        //     gets(keys);
        //     if(s == keys)
        //     printf("key value is %s\n",keys);
        // }
    }
}
int _kbhit()
{
    static const int STDIN = 0;
    static bool initialized = false;
    if (! initialized)
    {
        // Use termios to turn off line buffering
        termios term;
        tcgetattr(STDIN, &term);
        term.c_lflag &= ~ICANON;
        tcsetattr(STDIN, TCSANOW, &term);
        setbuf(stdin, NULL);
        initialized = true;
    }
    int bytesWaiting;
    ioctl(STDIN, FIONREAD, &bytesWaiting);
    return bytesWaiting;
}