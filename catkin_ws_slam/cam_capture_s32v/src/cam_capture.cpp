#include "parkinggo/parkinggo_common.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <log4cxx/logger.h>
#include <log4cxx/propertyconfigurator.h>

#include "message/groundPoint.h"
#include "message/slamParkingPoint.h"
#include "cam_capture/parkinggo_image.h"
#include "comm_mcu/mcu_msg.h"

#include "parkinggo/capture_v4l2.h"
#include "lut_lib/image_view_convert.h"
#include "video_capture.h"

using namespace cv;
using namespace std;
using namespace log4cxx;

static void get_rawdata(Mat& dst1, Mat& dst2, Mat& dst3, Mat& dst4);
static void publish_image(ros::Publisher& publisher, ros::Time timestamp, Mat& image, int& frame_no, short int* coord);
static void publish_image(image_transport::Publisher& publisher, Mat& image, int& frame_no, short int* coord);

static void save_parking_video(VideoWriter& encoder, FILE*& txt, bool& recording, char& status, short int* coord, Mat& front, Mat& back, Mat& left, Mat& right);

#define IMAGE_WIDTH           (1280)
#define IMAGE_HEIGHT          (720)
#define SLAM_IMAGE_WIDTH      (480)
#define SLAM_IMAGE_HEIGHT     (360)

//#define SAVE_PARKING_VIDEO

static Mat front_rawdata(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
static Mat rear_rawdata(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
static Mat left_rawdata(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
static Mat right_rawdata(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);

static unsigned char mem_video_front[IMAGE_WIDTH * IMAGE_HEIGHT * 2];
static unsigned char mem_video_back[IMAGE_WIDTH * IMAGE_HEIGHT * 2];
static unsigned char mem_video_left[IMAGE_WIDTH * IMAGE_HEIGHT * 2];
static unsigned char mem_video_right[IMAGE_WIDTH * IMAGE_HEIGHT * 2];

static pthread_mutex_t read_data_lock;
static pthread_mutex_t mcu_callback_lock;

static int frame_no = 0;
static char parking_status = 0;
static short int TimeStampex[3] = {0};

static PARKINGGO_CONFIG parkinggo_configs;

static LoggerPtr logger_parkinggo(Logger::getLogger("cam_capture"));

void mcu_msg_callback(const comm_mcu::mcu_msg& msg)
{
    LOG4CXX_TRACE(logger_parkinggo, "mcu_msg_callback: acquire mcu_callback_lock");
    pthread_mutex_lock(&mcu_callback_lock);

    parking_status = msg.car_parking_status;
    TimeStampex[0] = msg.TimeStampex[0];
    TimeStampex[1] = msg.TimeStampex[1];
    TimeStampex[2] = msg.TimeStampex[2];

    pthread_mutex_unlock(&mcu_callback_lock);
    LOG4CXX_TRACE(logger_parkinggo, "mcu_msg_callback: release mcu_callback_lock");
}

int main(int argc, char** argv)
{
    set_cpu(logger_parkinggo, CPU_ID_CAM_CAPTURE);

    load_config(logger_parkinggo, parkinggo_configs);

    LOG4CXX_INFO(logger_parkinggo, "Camera Capture start!");

    pthread_mutex_init(&read_data_lock, NULL);
    pthread_mutex_init(&mcu_callback_lock, NULL);

    // camera capture ros module
    ros::init(argc, argv, "cam_capture_s32v");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("mcu_msg", 10, mcu_msg_callback);

    // Old type images for SLAM
    image_transport::ImageTransport it_raw_front(nh);
    image_transport::Publisher pub_raw_front = it_raw_front.advertise("raw_front", 1);

    image_transport::ImageTransport it_raw_rear(nh);
    image_transport::Publisher pub_raw_rear = it_raw_rear.advertise("raw_rear", 1);

    image_transport::ImageTransport it_raw_left(nh);
    image_transport::Publisher pub_raw_left = it_raw_left.advertise("raw_left", 1);

    image_transport::ImageTransport it_raw_right(nh);
    image_transport::Publisher pub_raw_right = it_raw_right.advertise("raw_right", 1);

    // New type images for DNN
    ros::NodeHandle it_front;
    ros::Publisher pub_front = it_front.advertise<cam_capture::parkinggo_image>("front_image", 1);

    ros::NodeHandle it_rear;
    ros::Publisher pub_rear = it_rear.advertise<cam_capture::parkinggo_image>("back_image", 1);

    ros::NodeHandle it_left;
    ros::Publisher pub_left = it_left.advertise<cam_capture::parkinggo_image>("left_image", 1);

    ros::NodeHandle it_right;
    ros::Publisher pub_right = it_right.advertise<cam_capture::parkinggo_image>("right_image", 1);

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
    char status;

    if (parkinggo_configs.device == S32V)
    {
    }

    cv::VideoCapture* cap;
    if (parkinggo_configs.device == VIDEO)
    {
        cap = new cv::VideoCapture(parkinggo_configs.input_video.c_str());
    }

#ifdef SAVE_PARKING_VIDEO
    VideoWriter encoder;
    FILE* txt = NULL;
    bool recording = false;

    char path[255];
    sprintf(path, "mkdir -p %s\n", (parkinggo_configs.base_dir + "videos").c_str());
    system(path);    
#endif

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

            //imshow("stitch_image", stitch_image);
        }
        else
        {
            try 
            {
               long t1 = (double)cvGetTickCount();
               
               get_rawdata(dst1, dst2, dst3, dst4);
               
               long t2 = (double)cvGetTickCount();
               
               printf("Capture time =  %f ms \n",(t2-t1)/cvGetTickFrequency()/1000);
            }
            catch(...)
            {
                LOG4CXX_ERROR(logger_parkinggo, "main pthread: Exception catched");
            }
        }

        pthread_mutex_unlock(&read_data_lock);
        LOG4CXX_TRACE(logger_parkinggo, "main pthread: release read_data_lock");
        
        LOG4CXX_TRACE(logger_parkinggo, "main pthread: get_rawdata finish");
        LOG4CXX_TRACE(logger_parkinggo, "main pthread: get_rawdata time = " << ((double)cvGetTickCount() - t)/(cvGetTickFrequency()*1000) << " ms");

        front_rawdata.copyTo(raw_image_front);  
        rear_rawdata.copyTo(raw_image_rear);  
        left_rawdata.copyTo(raw_image_left);  
        right_rawdata.copyTo(raw_image_right);

        LOG4CXX_TRACE(logger_parkinggo, "main pthread: acquire mcu_callback_lock");
        pthread_mutex_lock(&mcu_callback_lock);
        
        coord[0] = TimeStampex[0];
        coord[1] = TimeStampex[1];
        coord[2] = TimeStampex[2];
        status = parking_status;

        pthread_mutex_unlock(&mcu_callback_lock);
        LOG4CXX_TRACE(logger_parkinggo, "main pthread: release mcu_callback_lock");

#ifdef  SAVE_PARKING_VIDEO
        save_parking_video(encoder, txt, recording, status, coord, raw_image_front, raw_image_rear, raw_image_left, raw_image_right);
#endif

        double fx = 0.0;
        double fy = 0.0;
        resize(front_rawdata, slam_image_front, Size(SLAM_IMAGE_WIDTH, SLAM_IMAGE_HEIGHT), fx, fy, CV_INTER_CUBIC); 
        resize(rear_rawdata, slam_image_rear, Size(SLAM_IMAGE_WIDTH, SLAM_IMAGE_HEIGHT), fx, fy, CV_INTER_CUBIC); 
        resize(left_rawdata, slam_image_left, Size(SLAM_IMAGE_WIDTH, SLAM_IMAGE_HEIGHT), fx, fy, CV_INTER_CUBIC); 
        resize(right_rawdata, slam_image_right, Size(SLAM_IMAGE_WIDTH, SLAM_IMAGE_HEIGHT), fx, fy, CV_INTER_CUBIC); 
      
        publish_image(pub_raw_front, slam_image_front, frame_no, coord);
        publish_image(pub_raw_rear, slam_image_rear, frame_no, coord);
        publish_image(pub_raw_left, slam_image_left, frame_no, coord);
        publish_image(pub_raw_right, slam_image_right, frame_no, coord);
      
        ros::Time timestamp = ros::Time::now();
        publish_image(pub_front, timestamp, raw_image_front, frame_no, coord);
        publish_image(pub_rear, timestamp, raw_image_rear, frame_no, coord);
        publish_image(pub_left, timestamp, raw_image_left, frame_no, coord);
        publish_image(pub_right, timestamp, raw_image_right, frame_no, coord);

        t = (double)cvGetTickCount() - t;
        LOG4CXX_TRACE(logger_parkinggo, "main pthread: run once time = " << t/(cvGetTickFrequency()*1000) << " ms");
        LOG4CXX_TRACE(logger_parkinggo, "main pthread: run once time finish");
        LOG4CXX_TRACE(logger_parkinggo, "main pthread: end <<<<<<<<<<<<<<<<<<<<<<<<<<<");

        char c = cv::waitKey(1);
        if (c == 27) // 'ESC'
        {
           break;
        }

        loop_rate.sleep();
        ros::spinOnce();
    }

    LOG4CXX_INFO(logger_parkinggo, "Camera Capture finish!");

    return 0;
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

static void save_parking_video(VideoWriter& encoder, FILE*& txt, bool& recording, char& status, short int* coord, Mat& front, Mat& back, Mat& left, Mat& right)
{
    if (status == 1 || status == 2 || status == 3)
    {
        if (status == 1 && !recording)
        {
            // Enter parking state, start recording
            // generate filename
            struct timeval tmnow;
            struct tm *tm;
            char time_buffer[30], usec_buffer[6];
            char filename_video[255], filename_txt[255];
            gettimeofday(&tmnow, NULL);
            tm = localtime(&tmnow.tv_sec);
            strftime(time_buffer, 30, "%Y%m%d-%H%M%S", tm);

            // Zero padding millisec
            std::ostringstream ss;
            ss << std::setw(3) << std::setfill('0') << (int) tmnow.tv_usec / 1000;
            sprintf(usec_buffer, "%s", ss.str().c_str());
            sprintf(filename_video, "%s/%s.avi", (parkinggo_configs.base_dir + "videos").c_str(), time_buffer);
            sprintf(filename_txt, "%s/%s.txt", (parkinggo_configs.base_dir + "videos").c_str(), time_buffer);

            encoder.open(filename_video, VideoWriter::fourcc('M', 'J', 'P', 'G'), 30.0, Size(IMAGE_WIDTH * 2, IMAGE_HEIGHT * 2), true);
            txt = fopen(filename_txt, "w");
            fprintf(txt, "status, x, y, z\n");

            recording = true;
        }

        if (!recording)
            return;

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
        fprintf(txt, "%d, %d, %d, %d\n", status, coord[0], coord[1], coord[2]);
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
