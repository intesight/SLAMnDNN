/**
* DSO ROS 接口
* 接受消息话题: image 类型: cam_capture::parkinggo_image
* 从雷达数据从消息数据读取
* 
* Author: 	Mario
* Data: 	2018/12/01
* Company:	Intesight.CO
*/

#include <locale.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "util/settings.h"
#include "FullSystem/FullSystem.h"
#include "FullSystem/PointCloudPublisher.h"
#include "util/Undistort.h"

#include "IOWrapper/Pangolin/PangolinDSOViewer.h"

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include "cv_bridge/cv_bridge.h"
#include "cam_capture/parkinggo_image.h"
#include "util/image_view_convert.h"

#include "radarTruePosi_core.h"

#include <dirent.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <opencv2/videoio.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include "dso_ros/keypointDSO.h"

#include <Eigen/Eigen>

// 读入去畸变参数
// std::string calib = "/home/intesight/project/LDSO/examples/Intesight/camera2.txt";/home/tangshuaishuai/intesight/catkin_dso_ros/arrowPass/2019-2-26
// MG
// std::string calib = "/home/intesight/dso/thirdparty/lut/camera_MG.txt";
// std::string calib = "/home/intesight/dso/thirdparty/lut/camera_MG _20190323.txt";
// RW /home/intesight/dso/thirdparty/lut
// std::string calib = "/home/intesight/dso/thirdparty/lut/camera_RW.txt";
std::string calib = "/home/intesight/dso/thirdparty/lut/camera_RW_cheng_4_15.txt";

std::string vignetteFile = "";
std::string gammaFile = "";

// 是否使用光度标定
// #define __USE_PHOTOMETRIC_CALIBRATION__

// 是否输出运行时间
// #define __OUTPUT_RUN_TIME__

using namespace dso;
using namespace std;

void parseArgument(char* arg)
{
	int option;
	char buf[1000];

	if(1==sscanf(arg,"quiet=%d",&option))
	{
		if(option==1)
		{
			setting_debugout_runquiet = true;
			printf("QUIET MODE, I'll shut up!\n");
		}
		return;
	}


	if(1==sscanf(arg,"nolog=%d",&option))
	{
		if(option==1)
		{
			setting_logStuff = false;
			printf("DISABLE LOGGING!\n");
		}
		return;
	}

	if(1==sscanf(arg,"nogui=%d",&option))
	{
		if(option==1)
		{
			disableAllDisplay = true;
			printf("NO GUI!\n");
		}
		return;
	}
	if(1==sscanf(arg,"nomt=%d",&option))
	{
		if(option==1)
		{
			multiThreading = false;
			printf("NO MultiThreading!\n");
		}
		return;
	}
	if(1==sscanf(arg,"calib=%s",buf))
	{
		calib = buf;
		printf("loading calibration from %s!\n", calib.c_str());
		return;
	}
	if(1==sscanf(arg,"vignette=%s",buf))
	{
		vignetteFile = buf;
		printf("loading vignette from %s!\n", vignetteFile.c_str());
		return;
	}

	if(1==sscanf(arg,"gamma=%s",buf))
	{
		gammaFile = buf;
		printf("loading gammaCalib from %s!\n", gammaFile.c_str());
		return;
	}

	printf("could not parse argument \"%s\"!!\n", arg);

}

FullSystem* fullSystem = 0;		///< FullSystem类
Undistort* undistorter = 0;		///< 去畸变类
int frameID = 0;				///< 帧数ID


/**
* SLAM系统图像输入,调用addActiveFrame.
*/
void vidCb(const cam_capture::parkinggo_image::ConstPtr &img, ros::Publisher& keypointPub)
{
	// ros::Duration(3).sleep();
cout<<"test is ok"<<endl;
cout<<"receive data = "<<img->coordX<<'\t'<<img->coordY<<'\t'<<img->coordZ<<endl;
	static int index =0;

	if(index < 3)
	{
		index++;
		return;
	}

	int radarMsg[3];
	double timestamp;
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img->image, sensor_msgs::image_encodings::MONO8);
	assert(cv_ptr->image.type() == CV_8U);
	assert(cv_ptr->image.channels() == 1);

	if(setting_fullResetRequested)
	{
		std::vector<IOWrap::Output3DWrapper*> wraps = fullSystem->outputWrapper;
		delete fullSystem;
		for(IOWrap::Output3DWrapper* ow : wraps) ow->reset();
		fullSystem = new FullSystem();
		fullSystem->linearizeOperation=false;
		fullSystem->outputWrapper = wraps;
		if(undistorter->photometricUndist != 0)
		fullSystem->setGammaFunction(undistorter->photometricUndist->getG());
		setting_fullResetRequested=false;
	}

	cv::Mat lutResult(480, 640, CV_8UC1);
	undistorter->undistortLut(cv_ptr->image, lutResult);	// 鱼眼图使用Lut去畸变
	MinimalImageB minImg(640,480,(unsigned char*)lutResult.data);
	
	// 第一个是图片,第二个是曝光时间,第三个是timestamp,第四个是factor;TODO:factor作用?
	ImageAndExposure* undistImg = undistorter->undistort<unsigned char>(&minImg, 1.5,0, 1.0f);

	imshow("xxx", lutResult);
	waitKey(10);
	cout<<"test is ok"<<endl;

	radarMsg[0] = img->coordX;
	radarMsg[1] = img->coordY;
	radarMsg[2] = img->coordZ;
	
#ifdef __OUTPUT_RUN_TIME__
	double t = (double)cvGetTickCount();
#endif

	// ＳLAM程序入口
	fullSystem->addActiveFrame(undistImg, frameID, radarMsg);
	std::cout<<dec<<"radarMsg20190326:"<<radarMsg[0]<<radarMsg[1]<<radarMsg[2]<<std::endl;

#ifdef __OUTPUT_RUN_TIME__
	t = (double)cvGetTickCount() - t;
	printf( "Time Consumption of runing [AddActiveFrame] Once = %gms\n", t/(cvGetTickFrequency()*1000) );
#endif

	frameID++;		// ROS中帧数计数
	delete undistImg;

	// 发布/DSO/keyframe话题
	bool savePCD=true;
	printf("fullSystem->needPubKeyFrame = %d\n",fullSystem->needPubKeyFrame);
	cout<<"fullSystem->needPubKeyFrame"<<fullSystem->needPubKeyFrame<<endl;
	if(fullSystem->needPubKeyFrame)
	{
		ROS_INFO("########## need publish Keyframe to GridMap ##########\n");
		PointCloudPublisher singlePublisher(fullSystem->getFrameHessian(), fullSystem->getFrameShell(), fullSystem->getK(), fullSystem->getPointCloud());
		std::vector<Eigen::Vector3f> keyPointData = singlePublisher.getKeyframe();
		Eigen::Vector3f currCamToWorld = singlePublisher.getCamToWorld();
		Eigen::Quaterniond currCamToWorldR = singlePublisher.getOrientation().back();

		dso_ros::keypointDSO keypointMsg;
		// keypointMsg.header = ros::Time::now ();
		keypointMsg.size = keyPointData.size();
		std::cout << "keyPointData的大小" << (int)keyPointData.size() << std::endl; 
		keypointMsg.camToWorldx = currCamToWorld[0];//currCamToWorld
		keypointMsg.camToWorldy = currCamToWorld[1];
		keypointMsg.camToWorldz = currCamToWorld[2];
		keypointMsg.camToWorldRw = (float)currCamToWorldR.w();//quaternion
		keypointMsg.camToWorldRx = (float)currCamToWorldR.x();
		keypointMsg.camToWorldRy = (float)currCamToWorldR.y();
		keypointMsg.camToWorldRz = (float)currCamToWorldR.z();

		for(int i = 0; i < keypointMsg.size; ++i){
			keypointMsg.x.push_back(keyPointData[i][0]);//pcl data
			keypointMsg.y.push_back(keyPointData[i][1]);
			keypointMsg.z.push_back(keyPointData[i][2]);
		}
		
		// add scale
		double scale = 0.0;
		fullSystem->get_slam_lamda_radar(&scale);
		printf("scale::%f\n",scale);
		// keypointMsg.scale = 4.25725;
		keypointMsg.scale = scale/1000.0;
		keypointMsg.frame_id = frameID;
		keypointMsg.coord_x = img->coordX;
		keypointMsg.coord_y = img->coordY;
		keypointMsg.coord_z = img->coordZ;
		printf("send msg scale = %f\n",keypointMsg.scale);
		keypointPub.publish(keypointMsg);
		fullSystem->needPubKeyFrame = false;	// 对于非关键帧,不需要发布关键点
		// singlePublisher.pclSaver();
		if(frameID > 150 && savePCD){
			// fullSystem->savePCD();
			savePCD=false;
		}
	}
}

/**
* 设置DSO默认参数
*/
void setDSOParameters(){
	setting_desiredImmatureDensity = 1500;
	setting_desiredPointDensity = 4000;
	setting_minFrames = 5;
	setting_maxFrames = 7;
	setting_maxOptIterations=6;
	setting_minOptIterations=1;
	setting_logStuff = false;
	setting_kfGlobalWeight = 1.3;

#ifdef __USE_PHOTOMETRIC_CALIBRATION__
	printf("MODE WITH CALIBRATION, vignette and exposure times!\n");
	setting_photometricCalibration = 2; // 模式2, apply inv. response & remove V.
    setting_affineOptModeA = 1e12; //-1: fix. >=0: optimize (with prior, if > 0).
    setting_affineOptModeB = 1e8; //-1: fix. >=0: optimize (with prior, if > 0).
#else
	printf("MODE WITH CALIBRATION, but without exposure times!\n");
    // mode
    setting_photometricCalibration = 0;
    setting_affineOptModeA = 0; //-1: fix. >=0: optimize (with prior, if > 0).
    setting_affineOptModeB = 0; //-1: fix. >=0: optimize (with prior, if > 0).
#endif
}

/**
* DSO ROS 主函数
*/
int main( int argc, char** argv )
{

	ros::init(argc, argv, "dso_live");
	for(int i=1; i<argc;i++) parseArgument(argv[i]);

	setDSOParameters();

	// =============== 获取去畸变参数 =================
    undistorter = Undistort::getUndistorterForFile(calib, gammaFile, vignetteFile);

    setGlobalCalib(
            (int)undistorter->getSize()[0],
            (int)undistorter->getSize()[1],
            undistorter->getK().cast<float>());

    fullSystem = new FullSystem();			// 新建FullSystem类
    fullSystem->linearizeOperation=true; 	// [Mario 2018/12/3: Track将等待Map线程结束]

    if(!disableAllDisplay)
	    fullSystem->outputWrapper.push_back(new IOWrap::PangolinDSOViewer(
	    		 (int)undistorter->getSize()[0],
	    		 (int)undistorter->getSize()[1]));

    if(undistorter->photometricUndist != 0)
    	fullSystem->setGammaFunction(undistorter->photometricUndist->getG());

    ros::NodeHandle nh;
	ros::Publisher keypointPub = nh.advertise<dso_ros::keypointDSO>("/DSO/keyframe", 10);
	// ros::Subscriber imgSub = nh.subscribe<cam_capture::parkinggo_image>("image", 1, boost::bind(&vidCb,_1,keypointPub));
	ros::Subscriber imgSub = nh.subscribe<cam_capture::parkinggo_image>("raw_image_radar", 1, boost::bind(&vidCb,_1,keypointPub));
    ros::spin();

    for(IOWrap::Output3DWrapper* ow : fullSystem->outputWrapper)
    {
        ow->join();		///< 绘图线程
        delete ow;
    }
    delete undistorter;
    delete fullSystem;
	return 0;
}