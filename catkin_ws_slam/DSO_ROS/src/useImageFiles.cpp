/**
* DSO ROS 跑数据库接口
* TODO: 直接使用DSO自带的 main_dso_pangolin.cpp
* 
* Author: 	Mario
* Data: 		2019/12/01
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
#include "IOWrapper/OutputWrapper/SampleOutputWrapper.h"

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include "cv_bridge/cv_bridge.h"
#include "parkinggo_image.h"
#include "util/image_view_convert.h"

#include <dirent.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <opencv2/videoio.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include "dso_ros/keypointDSO.h"        // DSO数据类型

#include <Eigen/Eigen>

char image_dir_name[100] = "/home/intesight/video/loop/ground_loop_photos/";

// std::string calib = "";
std::string calib = "/home/intesight/project/LDSO/examples/Intesight/camera2.txt";
std::string vignetteFile = "";
std::string gammaFile = "";

bool useSampleOutput = false; 	///< 不使用示例输出

// 从数据集读取ground truth数据
double i1,i2,i3,i4,i5,i6,i7,i8;
std::ifstream gourndtruth;
std::string dataPath = "/home/intesight/video/Debug/rgbd_dataset_freiburg3_long_office_household/groundtruth.txt";

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

	if(1==sscanf(arg,"sampleoutput=%d",&option))
	{
		if(option==1)
		{
			useSampleOutput = true;
			printf("USING SAMPLE OUTPUT WRAPPER!\n");
		}
		return;
	}

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

// Mario Debug [11/5]
int radarID = 0;				///< 雷达ID
int radarData[3] = {0,0,0};		///< 雷达数据

void sendImage2Dso(Mat rawimage)
{
	printf("将图像传递到 DSO SLAM中\n");
	Mat raw_image;
	cvtColor(rawimage, raw_image, CV_RGB2GRAY);
	assert(raw_image.type() == CV_8U);
	assert(raw_image.channels() == 1);

	double radarMsg[8];

	// Reset 重置系统
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

	MinimalImageB minImg((int)raw_image.cols, (int)raw_image.rows,(unsigned char*)raw_image.data);
	ImageAndExposure* undistImg = undistorter->undistort<unsigned char>(&minImg, 1,0, 1.0f);

	if(1){
		// 读取数据集中ground truth的数据
		std::string sreadingS;
		getline(gourndtruth, sreadingS);
		if(std::sscanf(sreadingS.c_str(), "%lf %lf %lf %lf %lf %lf %lf %lf ", &i1,&i2,&i3,&i4,&i5,&i6,&i7,&i8) == 8){
			// double quaternionNorm = sqrt(i5*i5+i6*i6+i7*i7+i8*i8); // 四元数归一化
			// std::cout << "###" << i5*i5+i6*i6+i7*i7+i8*i8 << std::endl;
			
			radarMsg[0] = i2/2.381;
			radarMsg[1] = i3/2.381;
			radarMsg[2] = i4/2.381;
			radarMsg[3] = i5;
			radarMsg[4] = i6;
			radarMsg[5] = i7;
			radarMsg[6] = i8;
			radarMsg[7] = i1;
			// std::cout << "x" << i5 << "y" << i6 << "z" << i7 << "w" << i8 << std::endl;
		}
		// ROS_INFO("Radar Time Stamp %lf",i1);
	}

#ifdef __OUTPUT_RUN_TIME__
	double t = (double)cvGetTickCount();
#endif

	// ================= SLAM程序入口 ======================
	fullSystem->addActiveFrame(undistImg, frameID, radarMsg);

#ifdef __OUTPUT_RUN_TIME__
	t = (double)cvGetTickCount() - t;
	printf( "Time Consumption of runing [AddActiveFrame] Once = %gms\n", t/(cvGetTickFrequency()*1000) );
#endif 

	frameID++;
	delete undistImg;

}

/**
* 设置DSO默认参数
*/
void setDSOParameters(){
	setting_desiredImmatureDensity = 3000;
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

	printf("cheng debug:calib name %s!\n", calib.c_str());

	// =============== 获取去畸变参数 =================
    undistorter = Undistort::getUndistorterForFile(calib, gammaFile, vignetteFile);

    setGlobalCalib(
            (int)undistorter->getSize()[0],
            (int)undistorter->getSize()[1],
            undistorter->getK().cast<float>());

    fullSystem = new FullSystem();
    fullSystem->linearizeOperation=true; // [Mario 2018/12/3: Track将等待Map线程结束]

    if(!disableAllDisplay)
	    fullSystem->outputWrapper.push_back(new IOWrap::PangolinDSOViewer(
	    		 (int)undistorter->getSize()[0],
	    		 (int)undistorter->getSize()[1]));

    if(useSampleOutput)
        fullSystem->outputWrapper.push_back(new IOWrap::SampleOutputWrapper());

    if(undistorter->photometricUndist != 0)
    	fullSystem->setGammaFunction(undistorter->photometricUndist->getG());

	// ================ 使用图片做为输入,调用sendImage2Dso =================

	// 读取 ground truth
	if(1){
		gourndtruth.open(dataPath);
		if (!gourndtruth) {
			std::cout << "Error! You need to change the path ###!!!" << std::endl;
		}
		std::string readingS;
		// getline(gourndtruth, readingS); // 忽略前三行
		// getline(gourndtruth, readingS);
		// getline(gourndtruth, readingS);
	}

    char *flow[65535];
    struct dirent* filename;
    DIR* dir = opendir(image_dir_name);
    int n;
    int i_name = 0;
    vector <string> files;
	Mat input_frame;

	int image_index = 0;

    while((filename = readdir(dir)) != NULL)
    {
        if(strcmp(filename->d_name, ".") == 0 ||
        strcmp(filename->d_name, "..") == 0)
        {
            continue;
        }

        int size = strlen(filename->d_name);

        flow[n] = (char*)malloc(sizeof(char)*size);

        strcpy(flow[n], filename->d_name);
        files.push_back(flow[n]);  
        n++;    

		image_index++;    
    }

    sort(files.begin(), files.end());

	while(1)
	{
		if (i_name < image_index)
		{
			char img_name[100];
			strcpy(img_name, image_dir_name);
			char string_name[100];
			strcat(img_name, files.at(i_name).c_str());

			input_frame = imread(img_name);

			// std::cout << "############ channels: " << input_frame.channels() << std::endl;
			// std::cout << "############ rows and cols: : " << input_frame.size()  << std::endl;
			// std::cout << "############" <<img_name<< std::endl;

			i_name++;

			// imshow("input_frame", input_frame);

			sendImage2Dso(input_frame);

		}
        // if(waitKey(10) >= 0)
        // {
        //     break;
        // }
		if(i_name == image_index)
			break;
	}

    for(IOWrap::Output3DWrapper* ow : fullSystem->outputWrapper)
    {
        ow->join();		///< 绘图线程
        delete ow;
    }
    delete undistorter;
    delete fullSystem;
	return 0;
}