// Vehicle Detection
#include "vehicle_detection.h"

#include "parkinggo/parkinggo_common.h"

#include <cv_bridge/cv_bridge.h>

#include "comm_mcu/mcu_msg.h"
#include "cam_capture/parkinggo_image.h"

#include "lut_lib/image_view_convert.h"

using namespace cv;
using namespace std;
using namespace log4cxx;
using namespace log4cxx::helpers;

#define IMAGE_WIDTH           (1280)
#define IMAGE_HEIGHT          (720)
#define PROJECTIVE_WIDTH      (640)
#define PROJECTIVE_HEIGHT     (480)

static Mat right_rawdata(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);

static pthread_mutex_t read_data_lock;

static PARKINGGO_CONFIG parkinggo_configs;

static LoggerPtr logger_parkinggo(Logger::getLogger("dnn_vehicle"));

static char parking_status = 0;
static int frame_no = 0;
static short int coords[3] = {0};

void mcu_msg_callback(const comm_mcu::mcu_msg& msg)
{
    parking_status = msg.car_parking_status;
}

void image_callback(const cam_capture::parkinggo_image img)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img.image, sensor_msgs::image_encodings::BGR8);
    frame_no = img.image.header.seq;
    coords[0] = img.coordX;
    coords[1] = img.coordY;
    coords[2] = img.coordZ;

    pthread_mutex_lock(&read_data_lock);

    cv_ptr->image.copyTo(right_rawdata);

    pthread_mutex_unlock(&read_data_lock);
}

int main(int argc, char** argv)
{
    set_cpu(logger_parkinggo, CPU_ID_DNN_VEHICLE);

    load_config(logger_parkinggo, parkinggo_configs);

    get_parameter(parkinggo_configs.lut_cfg);
    
    LOG4CXX_INFO(logger_parkinggo, "DNN Vehicle Detection start!");

    pthread_mutex_init(&read_data_lock, NULL);

    // Vehicle Detection ros module
    ros::init(argc, argv, "dnn_vehicle");

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("right_image", 1, image_callback);

    Mat undistort_image_lut(PROJECTIVE_HEIGHT, PROJECTIVE_WIDTH, CV_8UC3);
    Mat input_frame(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
    vector<vector<float> > vehicle_coordinates;

    // Initialize the Python Interpreter
    PythonInterpreter pyInterpreter;
    pyInterpreter.initialize();

    // Initialize Vehicle Detection module
    PythonModule pyVehicleModule;
    if (parkinggo_configs.vehicle_detection)
    {
        vehicle_detection_init(pyVehicleModule);
    }

    ros::Rate loop_rate(200);
    while(ros::ok())    
    {
        double t = (double)cvGetTickCount();
     
        LOG4CXX_TRACE(logger_parkinggo, "main pthread: start >>>>>>>>>>>>>>>>>>>>>>>>>>");      
        LOG4CXX_TRACE(logger_parkinggo, "main pthread: run once time start");

        // run vehicle detection on RUN state
        parking_status = 0x03;
        if (parking_status == 0x03)
        {
            if (parkinggo_configs.vehicle_detection)
            {
                pthread_mutex_lock(&read_data_lock);
                input_frame = right_rawdata;
                pthread_mutex_unlock(&read_data_lock);

                LOG4CXX_DEBUG(logger_parkinggo, "main pthread: lutTable_generate_image projective_view start");

                double t_lut_start = (double)cvGetTickCount();
                lutTable_generate_image(input_frame, undistort_image_lut, image_view_projective_e, view_right_e, fusion_min);
                //imshow("input image", input_frame);

                LOG4CXX_DEBUG(logger_parkinggo, "main pthread: lutTable_generate_image projective_view finish");

                double t_lut = (double)cvGetTickCount() - t_lut_start;
                LOG4CXX_DEBUG(logger_parkinggo, "main pthread: run lutTable_generate_image time = " << t_lut/(cvGetTickFrequency()*1000) << " ms");

                //imshow("undistort_image_lut", undistort_image_lut);

                // Call Vehicle Detection routine
                cv::Mat vehicles;
                double t_veh_start = (double)cvGetTickCount();

                LOG4CXX_DEBUG(logger_parkinggo, "main pthread: vehicle_detection start");

                int rtn = vehicle_detection(pyInterpreter, pyVehicleModule, undistort_image_lut, vehicles);

                LOG4CXX_DEBUG(logger_parkinggo, "main pthread: Vechiles = " << vehicles);
                double t_veh = (double)cvGetTickCount() - t_veh_start;
                LOG4CXX_DEBUG(logger_parkinggo, "main pthread: run vehicle detection time = " << t_veh/(cvGetTickFrequency()*1000) << " ms");
      
                LOG4CXX_DEBUG(logger_parkinggo, "main pthread: vehicle_detection finish");

                // Convert results from Mat to vector to match previous API
                vehicle_coordinates.clear();
                for (int i=0; i<vehicles.rows; i++)
                {
                    vector<float> veh;
                    veh.push_back(vehicles.at<float>(i, 2));
                    veh.push_back(vehicles.at<float>(i, 3));
                    veh.push_back(vehicles.at<float>(i, 4));
                    veh.push_back(vehicles.at<float>(i, 5)); 
                    veh.push_back(vehicles.at<float>(i, 0)); 
                    veh.push_back(vehicles.at<float>(i, 1));
                    vehicle_coordinates.push_back(veh);
                }
            }
        }

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

    // Stop Vehicle Detection Python Module
    if (parkinggo_configs.vehicle_detection)
    {
        vehicle_detection_stop(pyVehicleModule);
    }

    // Finish the Python Interpreter
    pyInterpreter.finalize();

    LOG4CXX_INFO(logger_parkinggo, "DNN Vehicle Detection finish!");

    return 0;
}
