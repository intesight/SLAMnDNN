// Parking Lots Detection
#include "parking_detection.h"

#include "parkinggo/parkinggo_common.h"

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include "comm_mcu/mcu_msg.h"
#include "cam_capture/parkinggo_image.h"
#include "dnn_lot/parking_lots.h"

#include "lut_lib/image_view_convert.h"

using namespace cv;
using namespace std;
using namespace log4cxx;
using namespace log4cxx::helpers;
using namespace cam_capture;
using namespace message_filters;
using namespace sensor_msgs;

typedef boost::shared_ptr<parkinggo_image const> ParkinggoImageConstPtr;

#define IMAGE_WIDTH           (1280)
#define IMAGE_HEIGHT          (720)
// #define MARKER_STITCH_WIDTH   (600)
#define MARKER_STITCH_WIDTH   (621)

#define MARKER_STITCH_HEIGHT  (720)
#define FRONT_BIRDVIEW_WIDTH  (621)
#define FRONT_BIRDVIEW_HEIGHT (279)
#define LEFT_BIRDVIEW_WIDTH   (279)
#define LEFT_BIRDVIEW_HEIGHT  (720)
#define RIGHT_BIRDVIEW_WIDTH  (279)
#define RIGHT_BIRDVIEW_HEIGHT (720)

#define LOT_MATCH_THRESHOLD_MM  800

//#define SAVE_PARKING_VIDEO

enum LOT_SELECTION_TYPE
{
    LOT_BEST,
    LOT_MATCH
};

static void show_parking_result(const cv::Mat& frame, const vector<vector<float> >& marker_coordinates);
static void send_parking_position(const ros::Publisher& pub, const vector< vector<int> >& worldlots, const float markerpoint[8], const int& frame_no, const short coords[3]);
static int markerpix2worldpoint(const vector<vector<float> >& marker_coordinates, const short int coords[3], vector< vector<int> >& worldlots, float markerpoint[8], LOT_SELECTION_TYPE type);
static void save_parking_video(VideoWriter& encoder, FILE*& txt, bool& recording, char& status, short int* coord, 
    Mat& front, Mat& back, Mat& left, Mat& right, const vector<vector<float> >& marker_coordinates);

static Mat front_rawdata(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
static Mat back_rawdata(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
static Mat left_rawdata(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
static Mat right_rawdata(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
static Mat birdview_stitch(LEFT_BIRDVIEW_HEIGHT, FRONT_BIRDVIEW_WIDTH * 2, CV_8UC3);
static vector< vector<int> > worldlots;
float selected_lot[8];
static int parking_status = 0;
static int parking_mode = 0;
static int frame_no = 0;
static short int coords[3] = {0};

static pthread_mutex_t read_data_lock;

static PARKINGGO_CONFIG parkinggo_configs;

static LoggerPtr logger_parkinggo(Logger::getLogger("dnn_lot"));

void mcu_msg_callback(const comm_mcu::mcu_msg& msg)
{
    parking_status = msg.car_parking_status;
    parking_mode = msg.parking_mode_select01;
}

void image_callback(const ParkinggoImageConstPtr& left, const ParkinggoImageConstPtr& right, const ParkinggoImageConstPtr& front, const ParkinggoImageConstPtr& back)
{
    pthread_mutex_lock(&read_data_lock);

    frame_no = left->image.header.seq;
    coords[0] = left->coordX;
    coords[1] = left->coordY;
    coords[2] = left->coordZ;

    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(left->image, sensor_msgs::image_encodings::BGR8);
    cv_ptr->image.copyTo(left_rawdata);

    cv_ptr = cv_bridge::toCvCopy(right->image, sensor_msgs::image_encodings::BGR8);
    cv_ptr->image.copyTo(right_rawdata);

    cv_ptr = cv_bridge::toCvCopy(front->image, sensor_msgs::image_encodings::BGR8);
    cv_ptr->image.copyTo(front_rawdata);

    cv_ptr = cv_bridge::toCvCopy(back->image, sensor_msgs::image_encodings::BGR8);
    cv_ptr->image.copyTo(back_rawdata);

    pthread_mutex_unlock(&read_data_lock);
}

int main(int argc, char** argv)
{
    set_cpu(logger_parkinggo, CPU_ID_DNN_LOT);

    load_config(logger_parkinggo, parkinggo_configs);

    get_parameter(parkinggo_configs.lut_cfg);

    LOG4CXX_INFO(logger_parkinggo, "DNN Parking Lot Detection start!");

    pthread_mutex_init(&read_data_lock, NULL);

    // Parking Lot Detection ros module
    ros::init(argc, argv, "dnn_lot");

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("mcu_msg", 10, mcu_msg_callback);    
    ros::Publisher pub_lot = nh.advertise<dnn_lot::parking_lots>("dnn_lots", 10);

    message_filters::Subscriber<parkinggo_image> camera_left_sub(nh, "left_image", 1);
    message_filters::Subscriber<parkinggo_image> camera_right_sub(nh, "right_image", 1);
    message_filters::Subscriber<parkinggo_image> camera_front_sub(nh, "front_image", 1);
    message_filters::Subscriber<parkinggo_image> camera_back_sub(nh, "back_image", 1);

    typedef sync_policies::ExactTime<parkinggo_image, parkinggo_image, parkinggo_image, parkinggo_image> ParkinggoSyncPolicy;
    // ExactTime takes a queue size as its constructor argument, hence ParkinggoSyncPolicy(3)
    Synchronizer<ParkinggoSyncPolicy> sync(ParkinggoSyncPolicy(3), camera_left_sub, camera_right_sub, camera_front_sub, camera_back_sub);
    sync.registerCallback(boost::bind(&image_callback, _1, _2, _3, _4));

    vector<vector<float> > marker_coordinates;
    Mat birdview_frame;

    // Initialize the Python Interpreter
    PythonInterpreter pyInterpreter;
    pyInterpreter.initialize();

    // Initialize Parking Lot Detection module
    PythonModule pyParkingModule;
    if (parkinggo_configs.parking_detection)
    {
        parking_detection_init(pyParkingModule);
    }

    if (parkinggo_configs.debug_display)
    {
        namedWindow("birdview");
        moveWindow("birdview", 200, 1150);
    }

    Mat front_image(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
    Mat left_image(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
    Mat right_image(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
    Mat back_image(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
    Mat birdview_image_lut(MARKER_STITCH_HEIGHT, MARKER_STITCH_WIDTH, CV_8UC3);

#ifdef SAVE_PARKING_VIDEO
    VideoWriter encoder;
    FILE* txt = NULL;
    bool recording = false;

    char path[255];
    sprintf(path, "mkdir -p %s\n", (parkinggo_configs.base_dir + "videos").c_str());
    system(path);    
#endif

    bool lot_found = false;
    ros::Rate loop_rate(200);
    while(ros::ok())    
    {
        double t = (double)cvGetTickCount();
     
        LOG4CXX_TRACE(logger_parkinggo, "main pthread: start >>>>>>>>>>>>>>>>>>>>>>>>>>");      
        LOG4CXX_TRACE(logger_parkinggo, "main pthread: run once time start");

        LOG4CXX_TRACE(logger_parkinggo, "main pthread: car_parking_status = " << parking_status);

        if (parkinggo_configs.device == VIDEO)
        {

            if (!lot_found)
            {
                parking_status = 0x01;
            }    
            else
            {
                parking_status = 0x03;
                lot_found = true;
   
             }
        }
        
        short coords_cur[3];
        if (parking_status == 0x01 || parking_status == 0x03)
        {
            double t_birdview_start = (double)cvGetTickCount();

            // run parking lots detection on SCAN and RUN state
            if (parking_status == 0x01 || parking_status == 0x03)
            {
                if (parkinggo_configs.parking_detection)
                {
                    LOG4CXX_DEBUG(logger_parkinggo, "main pthread: lutTable_generate_image birdview start");

                    pthread_mutex_lock(&read_data_lock);

                    int frame_no_cur = frame_no;
                    coords_cur[0] = coords[0];
                    coords_cur[1] = coords[1];
                    coords_cur[2] = coords[2];

                    pthread_mutex_unlock(&read_data_lock);

                    if (parking_status == 0x01)
                    {
                        // SCAN state, only right is used
                        right_rawdata.copyTo(right_image);
                        birdview_image_lut = lutTable_generate_stitchFusion(front_image, back_image, left_image, right_image, false, false, false, true);
                    }
                    else if (parking_status == 0x03)
                    {
                        // RUN state, left, right and back are used
                        front_rawdata.copyTo(front_image);
                        back_rawdata.copyTo(back_image);
                        left_rawdata.copyTo(left_image);
                        right_rawdata.copyTo(right_image);
                        birdview_image_lut = lutTable_generate_stitchFusion(front_image, back_image, left_image, right_image, true, true, true, true);
                    }
                
                    LOG4CXX_DEBUG(logger_parkinggo, "main pthread: lutTable_generate_image birdview finish");

                    birdview_frame = birdview_image_lut;
                    double t_birdview = (double)cvGetTickCount() - t_birdview_start;
                    LOG4CXX_DEBUG(logger_parkinggo, "main pthread: t_birdview time = " << t_birdview/(cvGetTickFrequency()*1000) << " ms");
            
                    // Call Parking Lot Detection routine
                    cv::Mat lots;

                    if (parkinggo_configs.debug_display)
                    {
                        Mat birdview_image = birdview_stitch(Rect(0, 0, FRONT_BIRDVIEW_WIDTH, LEFT_BIRDVIEW_HEIGHT));
                        birdview_frame.copyTo(birdview_image);
                        Mat birdview_stitch_resize;
                        resize(birdview_stitch, birdview_stitch_resize, Size(FRONT_BIRDVIEW_WIDTH, LEFT_BIRDVIEW_HEIGHT/2));
                        imshow("birdview", birdview_stitch_resize);
                    }

                    double t_lots_start = (double)cvGetTickCount();

                    LOG4CXX_DEBUG(logger_parkinggo, "main pthread: parking_detection start");

                    int rtn;
                    LOG4CXX_DEBUG(logger_parkinggo, "main pthread: parking_detection parking_mode: " << parking_mode);
                    
                    if (parking_status == 0x01 && parking_mode == 0x01)
                    {
                        // SCAN state with Vertical parking mode
                        rtn = parking_detection(pyInterpreter, pyParkingModule, birdview_image_lut, lots, true);
                    }
                    else
                    {
                        // SCAN state with Horizontal parking mode or RUN state
                        rtn = parking_detection(pyInterpreter, pyParkingModule, birdview_image_lut, lots, false);
                    }

                    LOG4CXX_DEBUG(logger_parkinggo, "main pthread: parking_detection finish");
                
                    LOG4CXX_DEBUG(logger_parkinggo, "main pthread: Lots = " << lots);
                    double t_lots = (double)cvGetTickCount() - t_lots_start;
                    LOG4CXX_DEBUG(logger_parkinggo, "main pthread: run parking_detection time = " << t_lots/(cvGetTickFrequency()*1000) << " ms");

                    // Convert results from Mat to vector to match previous API
                    marker_coordinates.clear();
                    for (int i=0; i<lots.rows; i++)
                    {
                        vector<float> lot;
                        lot.push_back(lots.at<float>(i, 0)); 
                        lot.push_back(lots.at<float>(i, 1));
                        lot.push_back(lots.at<float>(i, 2));
                        lot.push_back(lots.at<float>(i, 3));
                        lot.push_back(lots.at<float>(i, 4));
                        lot.push_back(lots.at<float>(i, 5));
                        lot.push_back(lots.at<float>(i, 6));
                        lot.push_back(lots.at<float>(i, 7));

                        marker_coordinates.push_back(lot);
                    }

                    if(marker_coordinates.size() > 0)
                    {
                        float markerpoint[8];
                        int match_rtn;
                        if (parking_status == 0x01)
                        {
                            // SCAN state, find best lot
                            match_rtn = markerpix2worldpoint(marker_coordinates, coords_cur, worldlots, markerpoint, LOT_BEST);
                            lot_found = true;
                        }
                        else if (parking_status == 0x03)
                        {
                            // RUN state, find matched lot
                            match_rtn = markerpix2worldpoint(marker_coordinates, coords_cur, worldlots, markerpoint, LOT_MATCH); 
                        }
                        
                        show_parking_result(birdview_image_lut, marker_coordinates);

                        // publish_radar(uart_data, worldlots, pub_radar);

                        if ((parking_status == 0x01 || parking_status == 0x03) && !match_rtn)
                        {
                            send_parking_position(pub_lot, worldlots, markerpoint, frame_no_cur, coords_cur);
                        }
                    }

                }
            }
        }

#ifdef  SAVE_PARKING_VIDEO
        save_parking_video(encoder, txt, recording, parking_status, coords_cur, front_image, back_image, left_image, right_image, marker_coordinates);
#endif

        char c = cv::waitKey(1);
        if (c == 27) // 'ESC'
        {
           break;
        }

        t = (double)cvGetTickCount() - t;
        LOG4CXX_TRACE(logger_parkinggo, "run once time = " << t/(cvGetTickFrequency()*1000) << " ms");

        LOG4CXX_TRACE(logger_parkinggo, "main pthread: run once time finish");
        LOG4CXX_TRACE(logger_parkinggo, "main pthread: end <<<<<<<<<<<<<<<<<<<<<<<<<<<");

        loop_rate.sleep();
        ros::spinOnce();
    }

    // Stop Parking Lot Detection Python Module
    if (parkinggo_configs.parking_detection)
    {
        parking_detection_stop(pyParkingModule);
    }

    // Finish the Python Interpreter
    pyInterpreter.finalize();

    LOG4CXX_INFO(logger_parkinggo, "DNN Parking Lot Detection finish!");

    return 0;
}

static void show_parking_result(const cv::Mat& frame, const vector<vector<float> >& marker_coordinates)
{
    float pointpix1[2]={0};
    float pointpix2[2]={0};
    float pointpix3[2]={0};
    float pointpix4[2]={0};

    for(int index_v = 0;index_v < marker_coordinates.size(); index_v++ )
    {
        pointpix1[0] = marker_coordinates[index_v][0]; //u
        pointpix1[1] = marker_coordinates[index_v][1]; //v

        pointpix2[0] = marker_coordinates[index_v][2]; //u
        pointpix2[1] = marker_coordinates[index_v][3]; //v

        pointpix3[0] = marker_coordinates[index_v][4]; //u
        pointpix3[1] = marker_coordinates[index_v][5]; //v

        pointpix4[0] = marker_coordinates[index_v][6]; //u
        pointpix4[1] = marker_coordinates[index_v][7]; //v

	    line(frame, Point(pointpix1[0], pointpix1[1]), Point(pointpix2[0], pointpix2[1]), Scalar(0,0,255),2);
        line(frame, Point(pointpix2[0], pointpix2[1]), Point(pointpix3[0], pointpix3[1]), Scalar(0,0,255),2);
        line(frame, Point(pointpix3[0], pointpix3[1]), Point(pointpix4[0], pointpix4[1]), Scalar(0,0,255),2);
        line(frame, Point(pointpix4[0], pointpix4[1]), Point(pointpix1[0], pointpix1[1]), Scalar(0,0,255),2);
    }

    imwrite("detected_lots.jpg", frame);

    if (parkinggo_configs.debug_display)
    {
        Mat lot_image = birdview_stitch(Rect(FRONT_BIRDVIEW_WIDTH, 0, FRONT_BIRDVIEW_WIDTH, LEFT_BIRDVIEW_HEIGHT));
        frame.copyTo(lot_image);
        Mat birdview_stitch_resize;
        resize(birdview_stitch, birdview_stitch_resize, Size(FRONT_BIRDVIEW_WIDTH, LEFT_BIRDVIEW_HEIGHT/2));
        imshow("birdview", birdview_stitch_resize);
    }
}

static void markpointsort(IN float markworldpooint[8], INOUT float marksortpoint[8])
{
    float temp_x = 0;
    float temp_y = 0;

    float x1 = markworldpooint[0];
    float x2 = markworldpooint[2];
    float x3 = markworldpooint[4];
    float x4 = markworldpooint[6];

    float y1 = markworldpooint[1];
    float y2 = markworldpooint[3];
    float y3 = markworldpooint[5];
    float y4 = markworldpooint[7];

    //float markworldpoointx[4] = {markworldpooint[0], markworldpooint[2],markworldpooint[4], markworldpoointx[6]};
    float markworldpoointx[4] = {x1, x2, x3, x4};
    float markworldpoointy[4] = {y1, y2, y3, y4};

    int n = 4;

    //冒泡排序，x绝对值从小到大，x方向靠近车的位子
    for(int i = 0; i < n-1;i++)
    {
        for(int j = 0; j < n-1-i; j++)
        {
            if(abs(markworldpoointx[j]) > abs(markworldpoointx[j+1]))
            {
                temp_x = markworldpoointx[j];
                temp_y = markworldpoointy[j];

                markworldpoointx[j] = markworldpoointx[j + 1];
                markworldpoointy[j] = markworldpoointy[j + 1];

                markworldpoointx[j + 1] = temp_x;
                markworldpoointy[j + 1] = temp_y;
            }
        }
    }

    //在x方向最靠近车的两个点，y的值大的，作为目标输出第一个点，另外一个作为目标输出第二个点
    if(markworldpoointy[0] > markworldpoointy[1])
    {
        marksortpoint[0] = markworldpoointx[0];
        marksortpoint[1] = markworldpoointy[0];

        marksortpoint[2] = markworldpoointx[1];
        marksortpoint[3] = markworldpoointy[1];
    }
    else
    {
        marksortpoint[0] = markworldpoointx[1];
        marksortpoint[1] = markworldpoointy[1];

        marksortpoint[2] = markworldpoointx[0];
        marksortpoint[3] = markworldpoointy[0];
    }

    //在x方向远离车的两个点，离第二个点距离较近的作为目标输出第三个点，另外一个作为目标第四个点,直角边小于斜边

    //markworldpooint第三个点离marksortpoint第二点距离
    float dis_y1 = abs(marksortpoint[3] - markworldpoointy[2]);
    float dis_x1 = abs(marksortpoint[2] - markworldpoointx[2]);
    float dis1 = sqrt(dis_x1 * dis_x1 + dis_y1 * dis_y1);

    //markworldpooint第四个点离marksortpoint第二点距离
    float dis_y2 = abs(marksortpoint[3] - markworldpoointy[3]);
    float dis_x2 = abs(marksortpoint[2] - markworldpoointx[3]);
    float dis2 = sqrt(dis_x2 * dis_x2 + dis_y2 * dis_y2);

    if(dis1 < dis2) //markworldpoointy第三个点在y方向上更靠近目标的目标输出第二个点
    {
        marksortpoint[4] = markworldpoointx[2];
        marksortpoint[5] = markworldpoointy[2];

        marksortpoint[6] = markworldpoointx[3];
        marksortpoint[7] = markworldpoointy[3];
    }
    else
    {
        marksortpoint[4] = markworldpoointx[3];
        marksortpoint[5] = markworldpoointy[3];

        marksortpoint[6] = markworldpoointx[2];
        marksortpoint[7] = markworldpoointy[2];
    }
    return ;
}

static string format_lot(const float points[8])
{
    stringstream ss;
    ss << "[";
    for (int i=0; i<4; i++)
    {
        ss << "(" << points[i*2] << ", " << points[i*2+1] << ") ";
    }
    ss << "]";

    return ss.str();
}

static string format_lot(vector<float> points)
{
    stringstream ss;
    ss << "[";
    for (int i=0; i<4; i++)
    {
        ss << "(" << points[i*2] << ", " << points[i*2+1] << ") ";
    }
    ss << "]";

    return ss.str();
}

static void coord_cc2oc(const short int coords[3], const float cc[8], float oc[8])
{
    float radian = (float) coords[2] / 1000;

    //LOG4CXX_DEBUG(logger_parkinggo, "ccoord_cc2oc, radian: " << radian << ", coords: " << "[" << coords[0] << ", " << coords[1] << "]" << ", cc: " << format_lot(cc));
    
    for (int i=0; i<4; i++)
    {
        float x = cc[i * 2 + 1];
        float y = -cc[i * 2];
        oc[i * 2]     = x * cos(radian) - y * sin(radian) + coords[0];
        oc[i * 2 + 1] = x * sin(radian) + y * cos(radian) + coords[1];
    }
}

static int markerpix2worldpoint(const vector<vector<float> >& marker_coordinates, const short int coords[3], vector< vector<int> >& worldlots, float markerpoint[8], LOT_SELECTION_TYPE type)
{
    float pointpix1[2] = {0};
    float pointpix2[2] = {0};
    float pointpix3[2] = {0};
    float pointpix4[2] = {0};
    float worldpoint[3];
    float markworldpooint[8];
    vector<vector<float> > marker_coordinates_sort;
    float marker_point_sort[8];

    int markersize = marker_coordinates.size();

    vector< vector<int> > worldlotstmp(markersize, vector<int>(8));

    worldlots = worldlotstmp;

    for(int index_v = 0; index_v < markersize; index_v++ )
    {
        //第一个点
        pointpix1[0] = marker_coordinates[index_v][0] - (MARKER_STITCH_WIDTH-RIGHT_BIRDVIEW_WIDTH); //u
        pointpix1[1] = marker_coordinates[index_v][1]; //v

        birdviewpix2worldpoint(pointpix1, view_right_e, worldpoint);

        worldlots[index_v][0] = (int)worldpoint[0];
        worldlots[index_v][1] = (int)worldpoint[1];

        markworldpooint[0] = worldpoint[0];
        markworldpooint[1] = worldpoint[1];

        //第二个点
        pointpix2[0] = marker_coordinates[index_v][2] - (MARKER_STITCH_WIDTH-RIGHT_BIRDVIEW_WIDTH); //u
        pointpix2[1] = marker_coordinates[index_v][3]; //v

        birdviewpix2worldpoint(pointpix2, view_right_e, worldpoint);
        markworldpooint[2] = worldpoint[0];
        markworldpooint[3] = worldpoint[1];

        worldlots[index_v][2] = (int)worldpoint[0];
        worldlots[index_v][3] = (int)worldpoint[1];

        //第三个点
        pointpix3[0] = marker_coordinates[index_v][4] - (MARKER_STITCH_WIDTH-RIGHT_BIRDVIEW_WIDTH); //u
        pointpix3[1] = marker_coordinates[index_v][5]; //v

        birdviewpix2worldpoint(pointpix3, view_right_e, worldpoint);
        markworldpooint[4] = worldpoint[0];
        markworldpooint[5] = worldpoint[1];

        worldlots[index_v][4] = (int)worldpoint[0];
        worldlots[index_v][5] = (int)worldpoint[1];

        //第四个点
        pointpix4[0] = marker_coordinates[index_v][6] - (MARKER_STITCH_WIDTH-RIGHT_BIRDVIEW_WIDTH); //u
        pointpix4[1] = marker_coordinates[index_v][7]; //v

        birdviewpix2worldpoint(pointpix4, view_right_e, worldpoint);
        markworldpooint[6] = worldpoint[0];
        markworldpooint[7] = worldpoint[1];

        worldlots[index_v][6] = (int)worldpoint[0];
        worldlots[index_v][7] = (int)worldpoint[1];

        markpointsort(markworldpooint, marker_point_sort);

        vector<float> markertmp;
        markertmp.push_back(marker_point_sort[0]); 
        markertmp.push_back(marker_point_sort[1]);
        markertmp.push_back(marker_point_sort[2]);
        markertmp.push_back(marker_point_sort[3]);
        markertmp.push_back(marker_point_sort[4]);
        markertmp.push_back(marker_point_sort[5]);
        markertmp.push_back(marker_point_sort[6]);
        markertmp.push_back(marker_point_sort[7]);

        marker_coordinates_sort.push_back(markertmp);
    }

    int marker_size = marker_coordinates_sort.size();

    if (type == LOT_BEST)
    {
        // find the best lot
        //当检测出超过2个车位
        if(marker_size > 2)
        {
            float markerpointy_max = 0.0;
            float markerpointy_min = 0.0;

            //找出所有车位第一个点的，y方向上的最大值与最小值
            for(int index = 0; index < marker_size; index++)
            {
                if(marker_coordinates_sort[index][1] > markerpointy_max)
                {
                    markerpointy_max = marker_coordinates_sort[index][1];
                }
                else if(marker_coordinates_sort[index][1] < markerpointy_min)
                {
                    markerpointy_min = marker_coordinates_sort[index][1];
                }
            }

            //根据最大值与最小值，求出平均值
            float markerpointy_mid = (markerpointy_max + markerpointy_min)/2;

            int mid_flag = 0;
            //y方向离平均值的距离
            float mid_distance = (marker_coordinates_sort[0][1] - markerpointy_mid) * (marker_coordinates_sort[0][1] - markerpointy_mid);
            //离平均值最近的为中间值，中间车位
            for(int index = 1; index < marker_size; index++)
            { 
                float distance = (marker_coordinates_sort[index][1] - markerpointy_mid) * (marker_coordinates_sort[index][1] - markerpointy_mid);
                if (distance < mid_distance)
                {
                    mid_distance = distance;
                    mid_flag = index;
                }
            }

            markerpoint[0] = marker_coordinates_sort[mid_flag][0];
            markerpoint[1] = marker_coordinates_sort[mid_flag][1];
            markerpoint[2] = marker_coordinates_sort[mid_flag][2];
            markerpoint[3] = marker_coordinates_sort[mid_flag][3];
            markerpoint[4] = marker_coordinates_sort[mid_flag][4];
            markerpoint[5] = marker_coordinates_sort[mid_flag][5];
            markerpoint[6] = marker_coordinates_sort[mid_flag][6];
            markerpoint[7] = marker_coordinates_sort[mid_flag][7];
        }
        else
        {
            markerpoint[0] = marker_coordinates_sort[0][0];
            markerpoint[1] = marker_coordinates_sort[0][1];
            markerpoint[2] = marker_coordinates_sort[0][2];
            markerpoint[3] = marker_coordinates_sort[0][3];
            markerpoint[4] = marker_coordinates_sort[0][4];
            markerpoint[5] = marker_coordinates_sort[0][5];
            markerpoint[6] = marker_coordinates_sort[0][6];
            markerpoint[7] = marker_coordinates_sort[0][7];
        }


        LOG4CXX_DEBUG(logger_parkinggo, "Lot Worldpoint: " << format_lot(markerpoint).c_str());

        coord_cc2oc(coords, markerpoint, selected_lot);

        return 0;
    }
    else if (type == LOT_MATCH)
    {
        // find lot matching previous scan
        float lot[8];
        for(int index = 0; index < marker_size; index++)
        {
            coord_cc2oc(coords, &marker_coordinates_sort[index][0], lot);
            
            LOG4CXX_DEBUG(logger_parkinggo, "finding matched lot, reference: " << format_lot(selected_lot).c_str());
            LOG4CXX_DEBUG(logger_parkinggo, "finding matched lot, current: " << format_lot(lot).c_str());

            if (abs(lot[0] - selected_lot[0]) <= LOT_MATCH_THRESHOLD_MM && 
                abs(lot[1] - selected_lot[1]) <= LOT_MATCH_THRESHOLD_MM &&
                abs(lot[2] - selected_lot[2]) <= LOT_MATCH_THRESHOLD_MM &&
                abs(lot[3] - selected_lot[3] <= LOT_MATCH_THRESHOLD_MM))
            {
                markerpoint[0] = marker_coordinates_sort[index][0];
                markerpoint[1] = marker_coordinates_sort[index][1];
                markerpoint[2] = marker_coordinates_sort[index][2];
                markerpoint[3] = marker_coordinates_sort[index][3];
                markerpoint[4] = marker_coordinates_sort[index][4];
                markerpoint[5] = marker_coordinates_sort[index][5];
                markerpoint[6] = marker_coordinates_sort[index][6];
                markerpoint[7] = marker_coordinates_sort[index][7];

                LOG4CXX_DEBUG(logger_parkinggo, "Lot Worldpoint: " << format_lot(markerpoint).c_str());

                LOG4CXX_DEBUG(logger_parkinggo, "found matched lot: " << format_lot(lot).c_str());
                return 0;
            }
        }

        return -1;
    }
}

void send_parking_position(const ros::Publisher& pub, const vector< vector<int> >& worldlots, const float markerpoint[8], const int& frame_no, const short coord[3])
{
    dnn_lot::parking_lots msg; 

    LOG4CXX_DEBUG(logger_parkinggo, "worldlots.size() = " << worldlots.size() );

    msg.frame_no = frame_no;
    msg.coordX = coord[0];
    msg.coordY = coord[1];
    msg.coordZ = coord[2];

    comm_mcu::lots_msg lot;
    lot.p0_x = markerpoint[0];
    lot.p0_y = markerpoint[1];
    lot.p1_x = markerpoint[2];
    lot.p1_y = markerpoint[3];
    lot.p2_x = markerpoint[4];
    lot.p2_y = markerpoint[5];
    lot.p3_x = markerpoint[6];
    lot.p3_y = markerpoint[7];
    msg.lots.push_back(lot);
    
    for (int i=0; i<worldlots.size(); i++)
    {
        if (worldlots[i][0] != (int)markerpoint[0] ||
            worldlots[i][1] != (int)markerpoint[1] ||
            worldlots[i][2] != (int)markerpoint[2] ||
            worldlots[i][3] != (int)markerpoint[3] ||
            worldlots[i][4] != (int)markerpoint[4] ||
            worldlots[i][5] != (int)markerpoint[5] ||
            worldlots[i][6] != (int)markerpoint[6] ||
            worldlots[i][7] != (int)markerpoint[7])
        {
            lot.p0_x = worldlots[i][0];
            lot.p0_y = worldlots[i][1]; 
            lot.p1_x = worldlots[i][2]; 
            lot.p1_y = worldlots[i][3]; 
            lot.p2_x = worldlots[i][4];
            lot.p2_y = worldlots[i][5];
            lot.p3_x = worldlots[i][6];
            lot.p3_y = worldlots[i][7];
            msg.lots.push_back(lot);
        }
    }

    pub.publish(msg);
}

void save_parking_video(VideoWriter& encoder, FILE*& txt, bool& recording, char& status, short int* coord, 
    Mat& front, Mat& back, Mat& left, Mat& right, const vector<vector<float> >& marker_coordinates)
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
            fprintf(txt, "status, x, y, z, p1_x, p1_y, p2_x, p2_y, p3_x, p3_y, p4_x, p4_y\n");

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
        for (int i=0; i<marker_coordinates.size(); i++)
            fprintf(txt, "%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n", status, coord[0], coord[1], coord[2], 
                    (int) marker_coordinates[i][0], (int) marker_coordinates[i][1], (int) marker_coordinates[i][2], (int)marker_coordinates[i][3],
                    (int) marker_coordinates[i][4], (int) marker_coordinates[i][5], (int) marker_coordinates[i][6], (int)marker_coordinates[i][7]);
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
