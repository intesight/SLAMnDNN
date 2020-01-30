// Semantic Segmentation
#include "semantic_segmentation.h"

#include "parkinggo/parkinggo_common.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

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

static void publish_image(image_transport::Publisher& publisher, Mat& image);

static Mat right_rawdata(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);

static pthread_mutex_t read_data_lock;

static PARKINGGO_CONFIG parkinggo_configs;

static LoggerPtr logger_parkinggo(Logger::getLogger("dnn_segmentation"));

void image_callback(const cam_capture::parkinggo_image img)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img.image, sensor_msgs::image_encodings::BGR8);
    int frame_no = img.image.header.seq;

    pthread_mutex_lock(&read_data_lock);

    cv_ptr->image.copyTo(right_rawdata);

    pthread_mutex_unlock(&read_data_lock);
}

int main(int argc, char** argv)
{
    set_cpu(logger_parkinggo, CPU_ID_DNN_SEGMENT);

    load_config(logger_parkinggo, parkinggo_configs);

    get_parameter(parkinggo_configs.lut_cfg);
    
    LOG4CXX_INFO(logger_parkinggo, "DNN Semantic Segmentation start!");

    pthread_mutex_init(&read_data_lock, NULL);

    // Semantic Segmentation ros module
    ros::init(argc, argv, "dnn_segmentation");

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("right_image", 1, image_callback);
    image_transport::ImageTransport semantic(nh);
    image_transport::Publisher pub_semantic = semantic.advertise("semantic_segmentation", 1);

    Mat input_frame(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
    Mat undistort_image_lut(PROJECTIVE_HEIGHT, PROJECTIVE_WIDTH, CV_8UC3);
    Mat imageSeg(PROJECTIVE_HEIGHT, PROJECTIVE_WIDTH, CV_8UC3);

    // Initialize the Python Interpreter
    PythonInterpreter pyInterpreter;
    pyInterpreter.initialize();

    // Initialize Semantic Segmentation module
    PythonModule pySegmentModule;
    if (parkinggo_configs.semantic_segmentation)
    {
        semantic_segmentation_init(pySegmentModule);
    }

    ros::Rate loop_rate(200);
    while(ros::ok())    
    {
        double t = (double)cvGetTickCount();
     
        LOG4CXX_DEBUG(logger_parkinggo, "main pthread: start >>>>>>>>>>>>>>>>>>>>>>>>>>");      
        LOG4CXX_DEBUG(logger_parkinggo, "main pthread: run once time start");

        pthread_mutex_lock(&read_data_lock);
        input_frame = right_rawdata;
        pthread_mutex_unlock(&read_data_lock);

        LOG4CXX_DEBUG(logger_parkinggo, "main pthread: lutTable_generate_image projective_view start");

        double t_lut_start = (double)cvGetTickCount();
        lutTable_generate_image(input_frame, undistort_image_lut, image_view_projective_e, view_right_e, fusion_min);
          
        LOG4CXX_DEBUG(logger_parkinggo, "main pthread: lutTable_generate_image projective_view finish");

        double t_lut = (double)cvGetTickCount() - t_lut_start;
        LOG4CXX_DEBUG(logger_parkinggo, "main pthread: run lutTable_generate_image time = " << t_lut/(cvGetTickFrequency()*1000) << " ms");

        //imshow("undistort_image_lut", undistort_image_lut);

        if (parkinggo_configs.semantic_segmentation)
        {
            // Call Semantic Segmentation routine
            double t_seg_start = (double)cvGetTickCount();

            LOG4CXX_DEBUG(logger_parkinggo, "main pthread: semantic_segmentation start");

            int rtn = semantic_segmentation(pyInterpreter, pySegmentModule, undistort_image_lut, imageSeg);
            double t_seg = (double)cvGetTickCount() - t_seg_start;
            LOG4CXX_DEBUG(logger_parkinggo, "run segmentation time = " << t_seg/(cvGetTickFrequency()*1000) << " ms");

            LOG4CXX_DEBUG(logger_parkinggo, "main pthread: semantic_segmentation finish");

            if (!rtn)
            {
                //imshow("segmentation", imageSeg);
                publish_image(pub_semantic, imageSeg);
            }
        }

        char c = cv::waitKey(1);
        if (c == 27) // 'ESC'
        {
           break;
        }

        t = (double)cvGetTickCount() - t;
        LOG4CXX_DEBUG(logger_parkinggo, "run once time = " << t/(cvGetTickFrequency()*1000) << " ms");
        LOG4CXX_DEBUG(logger_parkinggo, "main pthread: run once time finish");
        LOG4CXX_DEBUG(logger_parkinggo, "main pthread: end <<<<<<<<<<<<<<<<<<<<<<<<<<<");

        loop_rate.sleep();
        ros::spinOnce();
    }

    // Stop Semantic Segmentation Python Module
    if (parkinggo_configs.semantic_segmentation)
    {
         semantic_segmentation_stop(pySegmentModule);
    }

    // Finish the Python Interpreter
    pyInterpreter.finalize();

    LOG4CXX_INFO(logger_parkinggo, "DNN Semantic Segmentation finish!");

    return 0;
}

void publish_image(image_transport::Publisher& publisher, Mat& image)
{
    std_msgs::Header header = std_msgs::Header();
    header.stamp = ros::Time::now();
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
    publisher.publish(msg);
}
