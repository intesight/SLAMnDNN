#include "parkinggo/parkinggo_common.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <log4cxx/logger.h>
#include <log4cxx/propertyconfigurator.h>

#include "message/groundPoint.h"
#include "message/slamParkingPoint.h"

#include "cam_capture/parkinggo_image.h"
#include "comm_mcu/mcu_msg.h"

// #include "parkinggo_image.h"
// #include "mcu_msg.h"

#include "lut_lib/image_view_convert.h"
#include "video_capture.h"

using namespace cv;
using namespace std;
using namespace log4cxx;

static void save_parking_video(VideoWriter& encoder, FILE*& txt, bool& recording, char& status, short int* coord, short int* GPScoord, Mat& right);

#define IMAGE_WIDTH           (1280)
#define IMAGE_HEIGHT          (720)
#define SLAM_IMAGE_WIDTH      (480)
#define SLAM_IMAGE_HEIGHT     (360)

#define SAVE_PARKING_VIDEO

    bool recording = false;
static int front_fov_height, back_fov_height, left_fov_width, right_fov_width;

static Mat front_rawdata(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
static Mat rear_rawdata(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
static Mat left_rawdata(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
static cv::Mat right_rawdata(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);

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

static struct timeval tm_startImage_video;
static struct timeval tm_endImage_video;
static struct timeval tm_startRadar_video;
static struct timeval tm_endRadar_video;

static PARKINGGO_CONFIG parkinggo_configs;

static LoggerPtr logger_parkinggo(Logger::getLogger("save_video"));

static int save_flag = 0;
VideoWriter encoder;
FILE* txt = NULL;
char status;

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

    tm_startRadar_video.tv_sec = msg.radar_begin_sec;
    tm_startRadar_video.tv_usec = msg.radar_begin_usec;
    tm_endRadar_video.tv_sec = msg.radar_end_sec;
    tm_endRadar_video.tv_usec = msg.radar_end_usec;

    pthread_mutex_unlock(&mcu_callback_lock);
    LOG4CXX_TRACE(logger_parkinggo, "mcu_msg_callback: release mcu_callback_lock");
}

void image_callback(const cam_capture::parkinggo_image img)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img.image, sensor_msgs::image_encodings::BGR8);
    int frame_no = img.image.header.seq;

    pthread_mutex_lock(&read_data_lock);

    cv_ptr->image.copyTo(right_rawdata);

    pthread_mutex_unlock(&read_data_lock);

    short int coord[3];
    short int GPSCoord[3];
    // char status;
    // VideoWriter encoder;
    // FILE* txt = NULL;


    coord[0] = TimeStampex[0];
    coord[1] = TimeStampex[1];
    coord[2] = TimeStampex[2];

    GPSCoord[0] = GPS_coord[0];
    GPSCoord[1] = GPS_coord[1];
    GPSCoord[2] = GPS_coord[2];

    status = parking_status;

    tm_startImage_video.tv_sec = img.image_begin_sec;
    tm_startImage_video.tv_usec = img.image_begin_usec;
    tm_endImage_video.tv_sec = img.image_end_sec;
    tm_endImage_video.tv_usec = img.image_end_usec;

    save_flag = 1;

    LOG4CXX_TRACE(logger_parkinggo, "main pthread: release mcu_callback_lock");

    imshow("raw_image_right", right_rawdata);
    printf("cheng debug:status = %d \n", status);

}


int main(int argc, char** argv)
{
    set_cpu(logger_parkinggo, CPU_ID_CAM_CAPTURE);

    load_config(logger_parkinggo, parkinggo_configs);

    LOG4CXX_INFO(logger_parkinggo, "Camera Capture start!");

    pthread_t tStitchImage;
    
    pthread_mutex_init(&read_data_lock, NULL);
    pthread_mutex_init(&mcu_callback_lock, NULL);

    // camera capture ros module
    ros::init(argc, argv, "save_video");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("mcu_msg", 10, mcu_msg_callback);


    ros::Subscriber savevideo_sub = nh.subscribe("right_image", 3, image_callback);

    cv::Mat right_rawdata(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
    short int coord[3];
    short int GPSCoord[3];

    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setschedpolicy(&attr, SCHED_RR);

    char path[255];
    sprintf(path, "mkdir -p %s\n", (parkinggo_configs.base_dir + "videos").c_str());
    system(path);

    int pthread_stitch_flag = 0;

    ros::Rate loop_rate(33);
    while (ros::ok())    
    {
        double t = (double)cvGetTickCount();
     
        LOG4CXX_TRACE(logger_parkinggo, "main pthread: start >>>>>>>>>>>>>>>>>>>>>>>>>>");      

        loop_rate.sleep();
        ros::spinOnce();


        // coord[0] = TimeStampex[0];
        // coord[1] = TimeStampex[1];
        // coord[2] = TimeStampex[2];

        // GPSCoord[0] = GPS_coord[0];
        // GPSCoord[1] = GPS_coord[1];
        // GPSCoord[2] = GPS_coord[2];

        // status = parking_status;

        // LOG4CXX_TRACE(logger_parkinggo, "main pthread: release mcu_callback_lock");

        // static int record = 0;
        // if (status == 1 || record == 1)
        // {
        //     record = 1;
        //     status = 1;
        // }
        
        // imshow("raw_image_right", right_rawdata);
        // printf("cheng debug:status = %d \n", status);

        // double txx = (double)cvGetTickCount();

        // save_parking_video(encoder, txt, recording, status, coord, GPSCoord, right_rawdata);

        // txx = (double)cvGetTickCount() - txx;

        // printf("cheng debug: save_parking_video time = %f ms\n" , txx/(cvGetTickFrequency()*1000));

        if(save_flag == 1)
        {
            save_flag = 0;

            static int record = 0;
            if (status == 1 || record == 1)
            {
                record = 1;
                status = 1;
            }

             save_parking_video(encoder, txt, recording, status, coord, GPSCoord, right_rawdata);
        }

       

        t = (double)cvGetTickCount() - t;
        LOG4CXX_TRACE(logger_parkinggo, "main pthread: run once time = " << t/(cvGetTickFrequency()*1000) << " ms");
        LOG4CXX_TRACE(logger_parkinggo, "main pthread: end <<<<<<<<<<<<<<<<<<<<<<<<<<<");

        printf("cheng debug: cam_capture pthread: run once time = %f ms\n" , t/(cvGetTickFrequency()*1000));

        char c = cv::waitKey(1);
        if (c == 27) // 'ESC'
        {
           break;
        }

        // loop_rate.sleep();
        // ros::spinOnce();
    }

    LOG4CXX_INFO(logger_parkinggo, "Camera Capture finish!");

    return 0;
}


static void save_parking_video(VideoWriter& encoder, FILE*& txt, bool& recording, char& status, short int* coord,short int* GPScoord, Mat& right)
{
    if (status == 1 || status == 2 || status == 3)
    {
        // generate filename
        struct timeval tmnow;
        struct tm *tm;
        char time_buffer[30], usec_buffer[6];
        char filename_video[255], filename_txt[255];
        gettimeofday(&tmnow, NULL);
        tm = localtime(&tmnow.tv_sec);
        strftime(time_buffer, 30, "%Y%m%d-%H%M%S", tm);

        // std::cout << "time_buffer" << time_buffer << std::endl;

        // Zero padding millisec
        std::ostringstream ss;
        ss << std::setw(3) << std::setfill('0') << (int) tmnow.tv_usec / 1000;
        sprintf(usec_buffer, "%s", ss.str().c_str());

        // std::cout << "usec_buffer" << usec_buffer << std::endl;

////////////////////////////////////////////////////////////////////////////// image start time
        struct tm *tmStartImage;
        char time_StartImage[30], usec_StartImgae[6];
        tmStartImage = localtime(&tm_startImage_video.tv_sec);
        strftime(time_StartImage, 30, "%Y%m%d-%H%M%S",tmStartImage);

    //    std::cout << "time_StartImage " << time_StartImage << std::endl;


        // Zero padding millisec
        std::ostringstream ss_StartImage;
        ss_StartImage << std::setw(3) << std::setfill('0') << (int) tm_startImage_video.tv_usec / 1000;
        sprintf(usec_StartImgae, "%s", ss_StartImage.str().c_str());
    //    std::cout << "usec_StartImgae " << usec_StartImgae << std::endl;

////////////////////////////////////////   image end time

        struct tm *tmEndImage;
        char time_EndImage[30], usec_EndImgae[6];
        tmEndImage = localtime(&tm_endImage_video.tv_sec);
        strftime(time_EndImage, 30, "%Y%m%d-%H%M%S",tmEndImage);

        // std::cout << "time_EndtImage " << time_EndImage << std::endl;

        // Zero padding millisec
        std::ostringstream ss_EndImage;
        ss_EndImage << std::setw(3) << std::setfill('0') << (int) tm_endImage_video.tv_usec / 1000;
        sprintf(usec_EndImgae, "%s", ss_EndImage.str().c_str());
        // std::cout << "usec_EndImgae " << usec_EndImgae << std::endl;


//////////////////////////////////////// radar start time
        struct tm *tmStartRadar;
        char time_StartRadar[30], usec_StartRadar[6];
        tmStartRadar = localtime(&tm_startRadar_video.tv_sec);
        strftime(time_StartRadar, 30, "%Y%m%d-%H%M%S",tmStartRadar);

        // std::cout << "time_StartRadar " << time_StartRadar << std::endl;

        // Zero padding millisec
        std::ostringstream ss_StartRadar;
        ss_StartRadar << std::setw(3) << std::setfill('0') << (int) tm_startRadar_video.tv_usec / 1000;
        sprintf(usec_StartRadar, "%s", ss_StartRadar.str().c_str());
        // std::cout << "usec_StartRadar " << usec_StartRadar << std::endl;

//////////////////////////////////////// radar end time
        struct tm *tmEndRadar;
        char time_EndRadar[30], usec_EndRadar[6];
        tmEndRadar = localtime(&tm_endRadar_video.tv_sec);
        strftime(time_EndRadar, 30, "%Y%m%d-%H%M%S",tmEndRadar);

        // std::cout << "time_EndRadar " << time_EndRadar << std::endl;

        // Zero padding millisec
        std::ostringstream ss_EndtRadar;
        ss_EndtRadar << std::setw(3) << std::setfill('0') << (int) tm_endRadar_video.tv_usec / 1000;
        sprintf(usec_EndRadar, "%s", ss_EndtRadar.str().c_str());
        // std::cout << "usec_EndRadar " << usec_EndRadar << std::endl;


//////////////////////////////////////////////////////////////////////////////proc

        if (status == 1 && !recording)
        {
            // Enter parking state, start recording
            sprintf(filename_video, "%s/%s.avi", (parkinggo_configs.base_dir + "videos").c_str(), time_buffer);
            sprintf(filename_txt, "%s/%s.txt", (parkinggo_configs.base_dir + "videos").c_str(), time_buffer);

            encoder.open(filename_video, VideoWriter::fourcc('M', 'J', 'P', 'G'), 30.0, Size(IMAGE_WIDTH, IMAGE_HEIGHT), true);
           //  encoder.open(filename_video, VideoWriter::fourcc('M', 'J', 'P', 'G'), 30.0, Size(IMAGE_WIDTH * 2, IMAGE_HEIGHT * 2), true);
            txt = fopen(filename_txt, "w");
            fprintf(txt, "time, status, x, y, z, gpsx, gpsy, gpsz\n");

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
        encoder << right;
#endif

        fprintf(txt, "%s.%s, %d, %d, %d, %d,%d,%d,%d\n", time_buffer, usec_buffer, status, coord[0], coord[1], coord[2], GPScoord[0], GPScoord[1], GPScoord[2]);
        
        fprintf(txt, "startImgae:%s.%s, endImage:%s.%s, startRadar:%s.%s, endRadar:%s.%s\n", time_StartImage, usec_StartImgae, time_EndImage, usec_EndImgae, 
                                                                                                time_StartRadar, usec_StartRadar, time_EndRadar, usec_EndRadar);
    }
    else if ((status == 4 || status == 5) && recording)
    {
        // Quit parking state, stop recording
        encoder.release();
        if (txt != NULL)
        {
            fclose(txt);
            txt = NULL;
        }
        recording = false;
    }
}
