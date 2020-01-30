#ifndef __PARKINGGO_COMMON_H__
#define __PARKINGGO_COMMON_H__

#include <iostream>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <libgen.h>

#include <ros/ros.h>

#include "opencv2/opencv.hpp"

#include <log4cxx/logger.h>
#include <log4cxx/propertyconfigurator.h>

#define EXPERIMENTAL 1

#define CPU_ID_CAM_CAPTURE 0
#define CPU_ID_COMM_MCU    0
#define CPU_ID_DNN_LOT     1
#define CPU_ID_DNN_SEGMENT 2
#define CPU_ID_DNN_VEHICLE 3

enum INPUT_DEVICE
{
    S32V = 0, // S32V 4 channels
    CAM,      // USB camera
    VIDEO     // Video 4 channels
};

struct PARKINGGO_CONFIG
{
    std::string base_dir;      // app base directory
    INPUT_DEVICE device;       // input device
    std::string input_video;   // path of input video file (Used when device is video)
    std::string svm_dir;       // SVM parameters directory   
    std::string lut_cfg;       // lut lib config file
    bool debug_display;        // whether to display debug windows
    bool mcu_enable;           // whether to enable mcu communications
    bool parking_detection;    // whether to enable parking lots detection
    bool vehicle_detection;    // whether to enable vehicle detection
    bool semantic_segmentation;// whether to enable semantic segmentation
    bool front_slam;           // whether to enable front slam
    bool back_slam;            // whether to enable back slam
    bool left_slam;            // whether to enable left slam
    bool right_slam;           // whether to enable right slam
};

static void load_config(log4cxx::LoggerPtr& logger, PARKINGGO_CONFIG& parkinggo_configs)
{
    char result[PATH_MAX];
    size_t count = readlink("/proc/self/exe", result, PATH_MAX);
    const char *path;
    if (count != -1) 
    {
        path = dirname(result);
    }

    parkinggo_configs.base_dir = std::string(path) + "/../../";
    
    log4cxx::PropertyConfigurator::configure(parkinggo_configs.base_dir + "cfg/logging.cfg");

    cv::FileStorage fs;

    fs.open(parkinggo_configs.base_dir + "cfg/parkinggo_config.xml", cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        LOG4CXX_ERROR(logger, "Failed to open config file: " << parkinggo_configs.base_dir << "cfg/parkinggo_config.xml");
        return;
    }

    std::string str;
    fs["device"] >> str;
    if (str == "VIDEO")
        parkinggo_configs.device = VIDEO;
    else if (str == "CAM")
        parkinggo_configs.device = CAM;
    else
        parkinggo_configs.device = S32V;

    fs["input_video"] >> parkinggo_configs.input_video;
    parkinggo_configs.input_video = parkinggo_configs.base_dir + parkinggo_configs.input_video;

    fs["svm_dir"] >> parkinggo_configs.svm_dir;
    parkinggo_configs.svm_dir = parkinggo_configs.base_dir + parkinggo_configs.svm_dir + "/";

    fs["lut_cfg"] >> parkinggo_configs.lut_cfg;
    parkinggo_configs.lut_cfg = parkinggo_configs.base_dir + parkinggo_configs.lut_cfg;

    int val;
    fs["debug_display"] >> val;
    parkinggo_configs.debug_display = (val != 0);
        
    fs["mcu_enable"] >> val;
    parkinggo_configs.mcu_enable = (val != 0);
        
    fs["parking_detection"] >> val;
    parkinggo_configs.parking_detection = (val != 0);
        
    fs["vehicle_detection"] >> val;
    parkinggo_configs.vehicle_detection = (val != 0);
        
    fs["semantic_segmentation"] >> val;
    parkinggo_configs.semantic_segmentation = (val != 0);
    
    fs["front_slam"] >> val;
    parkinggo_configs.front_slam = (val != 0);
    
    fs["back_slam"] >> val;
    parkinggo_configs.back_slam = (val != 0);
    
    fs["left_slam"] >> val;
    parkinggo_configs.left_slam = (val != 0);
    
    fs["right_slam"] >> val;
    parkinggo_configs.right_slam = (val != 0);

    fs.release();
    
    LOG4CXX_INFO(logger, "ParkingGo Configuration");
    LOG4CXX_INFO(logger, "    base_dir:              " << parkinggo_configs.base_dir);
    LOG4CXX_INFO(logger, "    device:                " << str);
    LOG4CXX_INFO(logger, "    svm_dir:               " << parkinggo_configs.svm_dir);
    LOG4CXX_INFO(logger, "    lut_cfg:               " << parkinggo_configs.lut_cfg);
    LOG4CXX_INFO(logger, "    input_video:           " << parkinggo_configs.input_video);
    LOG4CXX_INFO(logger, "    debug_display:         " << parkinggo_configs.debug_display);
    LOG4CXX_INFO(logger, "    mcu_enable:            " << parkinggo_configs.mcu_enable);
    LOG4CXX_INFO(logger, "    parking_detection:     " << parkinggo_configs.parking_detection);
    LOG4CXX_INFO(logger, "    vehicle_detection:     " << parkinggo_configs.vehicle_detection);
    LOG4CXX_INFO(logger, "    semantic_segmentation: " << parkinggo_configs.semantic_segmentation);
    LOG4CXX_INFO(logger, "    front_slam:            " << parkinggo_configs.front_slam);
    LOG4CXX_INFO(logger, "    back_slam:             " << parkinggo_configs.back_slam);
    LOG4CXX_INFO(logger, "    left_slam:             " << parkinggo_configs.left_slam);
    LOG4CXX_INFO(logger, "    right_slam:            " << parkinggo_configs.right_slam);
}

static int set_cpu(log4cxx::LoggerPtr& logger, int cpu_id)  
{  
    cpu_set_t mask;  
    CPU_ZERO(&mask);    
    CPU_SET(cpu_id, &mask);  
  
    LOG4CXX_DEBUG(logger, "Set CPU Affinity, thread = " << pthread_self() << ", cpu_id = " << cpu_id);  
    if(-1 == pthread_setaffinity_np(pthread_self() ,sizeof(mask),&mask))  
    {  
         LOG4CXX_DEBUG(logger, "Set CPU Affinity, thread = " << pthread_self() << ", cpu_id = " << cpu_id << " failed ! ! ! ! ! ! ! ! ! !");  
         return -1;  
    } 

    return 0;  
} 

#endif // __PARKINGGO_COMMON_H__