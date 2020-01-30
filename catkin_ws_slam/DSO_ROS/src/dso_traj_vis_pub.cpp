/**
* DSO 前左右三个相机轨迹显示
* 接受消息话题: dso_live发出的 /DSO/keyframe 话题
* 发布三个traj为PointCloud数据
* TODO:
* 	1. 三个trajectory之间的转换
* 	2. scale实现自动计算
* 
* Author: 	Mario
* Data: 	2019/01/18
* Company:	Intesight.CO
*/

#include <ros/ros.h>
#include <stdio.h>
#include "dso_ros/keypointDSO.h"

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <sensor_msgs/PointCloud.h>
#include <opencv2/core/core.hpp>

using namespace std;

#define __USING_THREE_CAMS__    ///< 使用三个相机
#define __USING_RADAR_MSG__     ///< 读取雷达数据并发布点云数据

/**
* 从XML中读取相机车辆转换
*/
void readCamToCar(Eigen::Isometry3f& camToCar, std::string typeName);

/**
* 设置每个相机生成slam地图的尺度
*/
void setScale(float& scale);

/**
* DSO KeyFrame MSG 的 callback 函数
* 将轨迹以PointCloud形式转出
*/
void keyFramCb(dso_ros::keypointDSOConstPtr msg, std::vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f>>& camToCarV, 
					const float& scale, std::vector<Eigen::Vector3f>& dsoTraj,
                    ros::Publisher& trajectory_Pub);



enum camType{
    Front = 0,
    left,
    right
};

/**
* 主函数
*/
int main( int argc, char** argv )
{

	ros::init(argc, argv, "dso_traj_vis");

    Eigen::Isometry3f frontCamToCar = Eigen::Isometry3f::Identity();
    float frontScale = 7.20679;
    std::vector<Eigen::Vector3f > frontTraj;

    readCamToCar(frontCamToCar, "front_trans");
    setScale(frontScale);

#ifdef __USING_THREE_CAMS__
    Eigen::Isometry3f rightCamToCar = Eigen::Isometry3f::Identity();
    Eigen::Isometry3f leftCamToCar = Eigen::Isometry3f::Identity();  // 相机到世界坐标系转换
    float rightScale = 4.26315;
    float leftScale = 2.36 ;
    std::vector<Eigen::Vector3f > rightTraj;
    std::vector<Eigen::Vector3f > leftTraj;
    readCamToCar(rightCamToCar, "right_trans");
    readCamToCar(leftCamToCar, "left_trans");
    setScale(rightScale);
    setScale(leftScale);
#endif

    ros::NodeHandle nh;
    ros::Publisher frontTrajectory_Pub = nh.advertise<sensor_msgs::PointCloud> ("frontTrajectory", 1);
	std::vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f>> frontCamToCarV;
	frontCamToCarV.emplace_back(frontCamToCar);
	std::cout << "camToCar1" << frontCamToCar.matrix() << std::endl;
	std::cout << "frontCamToCarV" << frontCamToCarV[0].matrix() << std::endl;
	std::cout << "输入信号" << std::endl;
	ros::Subscriber keypointSubF = nh.subscribe<dso_ros::keypointDSO>("/DSO/keyframeF", 1, boost::bind(&keyFramCb, _1, frontCamToCarV, frontScale, frontTraj, frontTrajectory_Pub));

#ifdef __USING_THREE_CAMS__
    ros::Publisher rightTrajectory_Pub = nh.advertise<sensor_msgs::PointCloud> ("rightTrajectory", 1);
    ros::Publisher leftTrajectory_Pub = nh.advertise<sensor_msgs::PointCloud> ("leftTrajectory", 1);
    std::vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f>> rightCamToCarV;
    rightCamToCarV.emplace_back(rightCamToCar);
	std::vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f>> leftCamToCarV;
    leftCamToCarV.emplace_back(leftCamToCar);
    ros::Subscriber keypointSubR = nh.subscribe<dso_ros::keypointDSO>("/DSO/keyframeR", 1, boost::bind(&keyFramCb,_1,rightCamToCarV, rightScale, rightTraj, rightTrajectory_Pub));
    ros::Subscriber keypointSubL = nh.subscribe<dso_ros::keypointDSO>("/DSO/keyframeL", 1, boost::bind(&keyFramCb,_1,leftCamToCarV, leftScale, leftTraj, leftTrajectory_Pub));
#endif

#ifdef __USING_RADAR_MSG__

#endif

    ros::spin();
	return 0;
}

void readCamToCar(Eigen::Isometry3f& camToCar, std::string typeName){

    Eigen::Vector3f camToCarT;
    Eigen::Matrix3f camToCarR;

    std::string file;
    file = "/home/intesight/project/DSO/dso/thirdparty/lut/parameter_MG_10_31.xml";
    cv::FileStorage fs;
    fs.open(file, cv::FileStorage::READ);

    if (!fs.isOpened())
    {
        std::cout<<"open xml file ""failed "<< file << " failed " << std::endl;
        std::cout<<"Need to run in dso project fold, otherwise change the fold in code" << std::endl;
        exit(1);
        return;
    }

    cv::Mat trans;
    fs[typeName] >> trans;

    camToCarR(0,0) = trans.at<double>(0,0);
    camToCarR(0,1) = trans.at<double>(0,1);
    camToCarR(0,2) = trans.at<double>(0,2);
    camToCarR(1,0) = trans.at<double>(1,0);
    camToCarR(1,1) = trans.at<double>(1,1);
    camToCarR(1,2) = trans.at<double>(1,2);
    camToCarR(2,0) = trans.at<double>(2,0);
    camToCarR(2,1) = trans.at<double>(2,1);
    camToCarR(2,2) = trans.at<double>(2,2);

    camToCarT(0,0) = trans.at<double>(0,3)/1000;
    camToCarT(1,0) = trans.at<double>(1,3)/1000;
    camToCarT(2,0) = trans.at<double>(2,3)/1000;

    std::cout << "Reading Tcv From xml file: \n R: \n" << camToCarR << " \n T: \n" <<  camToCarT << std::endl;
    fs.release();

    camToCar.pretranslate(camToCarT);
	std::cout << "Reading ISOMETRYT: " << camToCar.matrix() << std::endl;

	camToCar.rotate(camToCarR);

	std::cout << "Reading ISOMETRY: " << camToCar.matrix() << std::endl;
}

void setScale(float& scale){
    scale = scale; 
}

void keyFramCb(dso_ros::keypointDSOConstPtr msg, 	
					std::vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f>>& camToCarV, 
					const float& scale, 
 					std::vector<Eigen::Vector3f >& dsoTraj,
                    ros::Publisher& trajectory_Pub){
   	Eigen::Isometry3f camToCar = camToCarV[0];
    Eigen::Vector3f camPose;
    camPose << msg->camToWorldx, msg->camToWorldy, msg->camToWorldz;
	ROS_INFO("CAM POSE IN MAP: x value %f, y value %f, z value %f", camPose[0], camPose[1], camPose[2]);
    camPose *= scale;
	ROS_INFO("CAM POSE AFTER SCALE: x value %f, y value %f, z value %f", camPose[0], camPose[1], camPose[2]);
	std::cout << "R,T" <<  camToCar.matrix() << std::endl;
	std::cout << "R,T Inverse" <<  camToCar.inverse().matrix() << std::endl;
    Eigen::Vector3f camWorld = camToCar.inverse() * camPose;
    ROS_INFO("CAM POSE IN WORLD: x value %f, y value %f, z value %f", camWorld[0], camWorld[1], camWorld[2]);
    dsoTraj.emplace_back(Eigen::Vector3f(camWorld[0], camWorld[1], camWorld[2]));
    
    sensor_msgs::PointCloud pointCloudmsg;
    pointCloudmsg.header.stamp = ros::Time::now();
    pointCloudmsg.header.frame_id = "map";
    pointCloudmsg.points.resize(dsoTraj.size());
    for(unsigned int i = 0;i < dsoTraj.size(); ++i){
        pointCloudmsg.points[i].x = dsoTraj[i][0];
        pointCloudmsg.points[i].y = dsoTraj[i][1];
        pointCloudmsg.points[i].z = dsoTraj[i][2];
    }
    trajectory_Pub.publish(pointCloudmsg);
	std::cout << "接受信号,输出点云" << std::endl;
}