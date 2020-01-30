#include <iostream>


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "cam_capture/parkinggo_image.h"
//OPENCV
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

#define _MG_TESTING_
//  /home/intesight/catkin_ws_slam/devel/videos/20190312/front1/20190312-104208-front.avi
//  /home/intesight/catkin_ws_slam/devel/videos/20190315-154142-front.avi
//20190312-105531-front

//20190315-152654-front
#ifdef _MG_TESTING_
    string videoFile =        "/home/intesight/catkin_ws_slam/devel/videos/20190312/front4/20190312-105724-front.avi";
    char position_name[100] = "/home/intesight/catkin_ws_slam/devel/videos/20190312/front4/20190312-105724.txt";
#else
    string videoFile =        "/home/intesight/catkin_ws_slam/devel/videos/20190315-154142-front.avi";
    char position_name[100] = "/home/intesight/catkin_ws_slam/devel/videos/20190315-154142.txt";
#endif


int main(int argc, char *argv[])
{
    FILE *fp_position = NULL;
    cv::VideoCapture cap(videoFile);
    if (!cap.isOpened())
    {
        /* code for True */
        std::cout<<"can not open video: "<<videoFile<<std::endl;
    }    
    char buf[1024];
    fp_position = fopen(position_name,"r");
    fgets(buf,sizeof(buf),fp_position); // ignore a line
    
    // slam ros module
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    ros::Publisher pub_front = nh.advertise<cam_capture::parkinggo_image>("raw_image_radar", 1);
    ros::Rate loop_rate(30);
    while (ros::ok())
    {
        /* code for loop body */
        Mat input_frame;
        cap >> input_frame;
        if(input_frame.empty())
        {
            cout<<"cap video end!"<<endl;
            break;
        }
        imshow("input_frame",input_frame);
        waitKey(10);
    }
    return 0;
}
