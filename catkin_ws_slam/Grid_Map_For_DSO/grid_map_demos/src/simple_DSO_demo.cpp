/*
 * DSO GridMap 
 * 将DSO中的点云在GridMap中显示出来
 * TODO: 
 *  构造成类,方便数据传输
 *  最小高度
 *  Created on: Nov 01, 2018
 *      Author: Mario
 *   Institute: Intesight Co.
 */
 #include <stdio.h>
 #include <sys/socket.h>
 #include <sys/types.h>
 #include <stdlib.h>
 #include <netinet/in.h>
 #include <errno.h>
 #include <string.h>
 #include <arpa/inet.h>
 #include <unistd.h>
 #include <sys/time.h>
 #include <time.h>
#include <pthread.h>
//proc

#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include "grid_map_msgs/keyframeMsg.h"
// #include "grid_map_msgs/keypointDSO.h"
#include "dso_ros/keypointDSO.h"
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <stdlib.h>
#include <sophus/sim3.hpp>
#include <sophus/se3.hpp>
#include <sensor_msgs/PointCloud.h>

#include <iostream>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <vector>
using namespace grid_map;
using namespace std;
using namespace cv;


/***********global para***************/
#define NUM_THREADS 1
int sendData[6] = {1,2,3,1,2,3};
char* servAddr = "192.168.4.200";
float Car_Width = 1.919;
float Car_Height = 4.678;
float Rear_Axle = 3.752;
//搜索距离设定
float obstacleX = 2;
float obstacleY = 6;
/**************end*******************/

bool usingDebugInfo = false;                ///< 显示Debug信息
bool usingPointCloud = false;               ///< 显示前相机点云数据
bool usingObstacleDetectFront = false;       ///< 显示前相机障碍物
bool usingObstacleDetect = true;            ///< 显示全局障碍物
bool usingTrajectory = true;                ///< 显示相机轨迹
bool usingCarPose = true;                   ///< 显示车辆5个点的位置

struct PubKeyFrameStruct {
    float x,y,z;
};

typedef struct Radar_Map_Data{
    float radar_data[3];
    float calculate_data[3];
}Radar_Map_S,*Radar_Map_P;

Radar_Map_S Receive_Data;


// ================= 相机外参 ===============
//    // 右相机外参
//    Eigen::Matrix3f rightCamToWorldRd;
//    rightCamToWorldRd << -0.077477202, -0.996993704, -0.000915140, -0.681557303, 0.053634259, -0.729796553,
//            0.727651652, -0.055918875, -0.683663772;
//    Eigen::Vector3f rightCamToWorldTd;
//    rightCamToWorldTd << 1.993103297369, 1.241970110547, 0.030066911255;

// 前相机外参
Eigen::Matrix3f rightCamToWorldRd;
Eigen::Vector3f rightCamToWorldTd;

/**
* 发布全局障碍物点
*/
void pubGlobalObstacle(GridMap& map_, ros::Publisher& pointCloudPub, const Eigen::Vector3f& carMapPose, const Eigen::Matrix3f& currR);

/**
* 发布全局障碍物点,线性遍历
*/
void pubGlobalObstacleLine(GridMap& map_, ros::Publisher& pointCloudPub, const Eigen::Vector3f& carMapPose, const Eigen::Matrix3f& currR);

/**
* keypointDSO的回调函数
* 将msg信息转换成GridMap
*/
//dso_ros::keypointDSO
//grid_map_msgs::keypointDSOConstPtr
void frameCb(dso_ros::keypointDSOConstPtr msg, GridMap *map, ros::Publisher& pointCloudPub, std::vector<Eigen::Vector3f>& dsoTraj, 
                    ros::Publisher& trajectory_Pub);
/**
* 向polygon里面增加点
*/ 
inline void polygonAdd(GridMap* map, grid_map::Polygon& polygon, const Eigen::Vector3f& carMapPose, const Eigen::Matrix3f& currR, Eigen::Vector3f point);


inline void convertCarToMap(Eigen::Vector3f point, Position &position, const Eigen::Vector3f& carMapPose, const Eigen::Matrix3f& currR);


void sysTime(void);

//20190404
// #define size 6
// #define PORT 10004
 void* server(void* args);

/**
 * DSO的地图处理主函数
 * @param argc
 * @param argv
 * @return
 */

int main(int argc, char **argv) {

    // 前相机 MG
    // rightCamToWorldRd << 0.998389641, -0.033445164, -0.045820805, -0.056713332, -0.569767769, -0.819846502,
    //     0.001312683, 0.821124905, -0.570747025;
    // rightCamToWorldTd << 0.135201065099, 2.460345053298, -2.657716463713;
    // rightCamToWorldRd << 0.998742954,-0.041008111,-0.028824412, -0.046341068,-0.536223581,-0.842802929,
    //     0.019105427,0.843079241,-0.537449883;
    // rightCamToWorldTd << 0.164880265832, 2.367901552570, -2.743990605984;
    //0.998742954   -0.041008111   -0.028824412  164.880265832
    //        -0.046341068   -0.536223581   -0.842802929 2367.901552570
    // 0.019105427    0.843079241   -0.537449883 -2743.990605984
    // 前相机 RW
    rightCamToWorldRd <<0.999996148, -0.001069859, 0.002561065,
                        0.001743915, -0.475654544, -0.879630441,
                        0.002159263, 0.879631519, -0.475650846;
    rightCamToWorldTd << 0.008534335292, 2.295221704876,  -2.925798434440;

    // 新建GridMap
    GridMap map({"maxHeight", "minHeight", "pointNum", "OccupancyMap", "Trajectory"});
    map.setFrameId("map");

//    map.setGeometry(Length(25, 15), 0.1);   // 长25,宽15,分辨率0.1m, 针对右相机倒车情况
    map.setGeometry(Length(100, 50), 0.1);   // 长50,宽30,分辨率0.1m 针对前相机直行情况

    // 初始化GridMap
    for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
        Position position;
        map.getPosition(*it, position);
        map.at("maxHeight", *it) = 0;
        map.at("minHeight", *it) = 0;
        map.at("pointNum", *it) = 0;
        map.at("OccupancyMap", *it) = 0;
        map.at("Trajectory", *it) = 0;
    }

    // 保存DSO路径
    std::vector<Eigen::Vector3f> dsoTraj;

    // 初始化rosnode,发布,接收
    ros::init(argc, argv, "DSO_GridMap_Demo");
    ros::NodeHandle nh("~");

    // 1. 发布grid_map话题
    ros::Publisher publisher = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
    // 2. 发布trajectory话题
    ros::Publisher trajectory_Pub = nh.advertise<sensor_msgs::PointCloud> ("dso_Trajectory", 1);
    // 3. 发布dso_pointcloud话题
    ros::Publisher pointCloud_Pub = nh.advertise<sensor_msgs::PointCloud> ("dso_PointCloud", 1);

    // A. 接收dso_trajectory
    ros::Subscriber keyFrames_Sub = nh.subscribe<dso_ros::keypointDSO>("/DSO/keyframe", 20,
                                                                             boost::bind(&frameCb, _1, &map, pointCloud_Pub, dsoTraj, trajectory_Pub));
    // 设定频率30Hz
    ros::Rate rate(30.0);

    //参数依次是：创建的线程id，线程参数，调用的函数，传入的函数参数
    // pthread_t tids[NUM_THREADS];
    // int ret = pthread_create(&tids[0], NULL, server, NULL);
    // if (ret != 0)
    // {
    //     cout << "pthread_create error: error_code=" << ret << endl;
    // }

    while (nh.ok()) {

        // 设置GridMap时间
        ros::Time time = ros::Time::now();
        map.setTimestamp(time.toNSec());

        // 将grid_map转化为ros message
        grid_map_msgs::GridMap message;
        GridMapRosConverter::toMessage(map, message);

        // 发布 grid_map message
        publisher.publish(message);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

/**
* Polygon搜索,发布障碍物点
*/
void pubGlobalObstacle(GridMap* map_, ros::Publisher& pointCloudPub, const Eigen::Vector3f& carMapPose, const Eigen::Matrix3f& currR){

    static bool recording = false;
    static FILE *fp_file;
    static char file_name[255];
    ROS_INFO("carMapPose: %f, %f, %f", carMapPose[0], carMapPose[1], carMapPose[2]);
    //保存radar与dso车辆轨迹数据
    printf("sss %d\n",recording);
    if(!recording)
    {
        sprintf(file_name, "/home/intesight/catkin_ws_slam/Trajectory.txt");
        fp_file = fopen(file_name, "w+");
        if(fp_file == NULL)
           printf("can not open *.txt\n");
        fprintf(fp_file, "radar_x, radar_y, radar_z : dso_x,dso_y,dso_z\n");
        printf("fp:%s\n",file_name);
        recording = true;
    }
    printf("write data\n");
    //写入历史轨迹数据
    // fprintf(fp_file, "radar_x, radar_y, radar_z : dso_x,dso_y,dso_z\n");
    fprintf(fp_file, "%10.8f,%10.8f,%10.8f,%10.8f,%10.8f,%10.8f\n ",  Receive_Data.radar_data[0], Receive_Data.radar_data[1],Receive_Data.radar_data[2],
        Receive_Data.calculate_data[0],Receive_Data.calculate_data[1],Receive_Data.calculate_data[2]);
    // 障碍物距离设定

    float obstacleZmin = 0.1;
    float obstacleZmax = 3;

    // 2019-01-25 暂时发布点云数据显示
    sensor_msgs::PointCloud pointCloudmsg;
    pointCloudmsg.header.stamp = ros::Time::now();
    pointCloudmsg.header.frame_id = "map";

    // polygon(菱形) 遍历
    grid_map::Polygon polygon;
    polygon.setFrameId(map_->getFrameId());
    polygonAdd(map_, polygon, carMapPose, currR, Eigen::Vector3f(Car_Width/2  + obstacleX, 3.594  + obstacleY, 0));
    polygonAdd(map_, polygon, carMapPose, currR, Eigen::Vector3f(-Car_Width/2 - obstacleX, 3.594  + obstacleY, 0));
    polygonAdd(map_, polygon, carMapPose, currR, Eigen::Vector3f(Car_Width/2  + obstacleX, -1.025 - obstacleY, 0));
    polygonAdd(map_, polygon, carMapPose, currR, Eigen::Vector3f(-Car_Width/2 - obstacleX, -1.025 - obstacleY, 0));

    int pointI = 0;
    int pointNum = 0;
    for (grid_map::PolygonIterator iterator(*map_, polygon); !iterator.isPastEnd(); ++iterator) {
        float singlePointHeight = map_->at("maxHeight", *iterator);  ///< Polygon当前位置的最大高度
        if(singlePointHeight > obstacleZmin){
            ++pointNum;
        }
    }
    ROS_INFO("The detected number of points: %d", pointNum);
    pointCloudmsg.points.resize(pointNum + 2);  

    for (grid_map::PolygonIterator iterator(*map_, polygon); !iterator.isPastEnd(); ++iterator) {

        float singlePointHeight = map_->at("maxHeight", *iterator);
        if(singlePointHeight > obstacleZmin){
            // map_.at("OccupancyMap", *iterator) = 5.0;
            const Index tmp = *(iterator);
            Position position1;
            map_->getPosition(tmp, position1);
            ROS_INFO("the value of iterator: %f, %f", position1[0], position1[1]);
            pointCloudmsg.points[pointI].x = position1[0];
            pointCloudmsg.points[pointI].y = position1[1];
            pointCloudmsg.points[pointI].z = 2.5;
            ++pointI;
        }
    }

    ROS_INFO("The number of Obstacles: %d", pointNum);
    pointCloudPub.publish(pointCloudmsg);

}

/**
* 线性搜索,发布障碍物点
*/
void pubGlobalObstacleLine(const GridMap* map_, ros::Publisher& pointCloudPub, const Eigen::Vector3f& carMapPose, const Eigen::Matrix3f& currR)
{

    static bool recording = false;
    static FILE *fp_file;
    char file_name[255];
    ROS_INFO("carMapPose: %f, %f, %f", carMapPose[0], carMapPose[1], carMapPose[2]);
    //保存radar与dso车辆轨迹数据
    if(!recording)
    {
        sprintf(file_name, "/home/intesight/catkin_ws_slam/Trajectory.txt");
        fp_file = fopen(file_name, "w+");
        if(fp_file == NULL)
           printf("can not open *.txt\n");
        fprintf(fp_file, "radar_x, radar_y, radar_z : dso_x,dso_y,dso_z\n");
        recording = true;
    }
    printf("enter pubobstaleLine function0\n");
    //写入历史轨迹数据
    // fprintf(fp_file, "radar_x %12.5f, radar_y %12.5f, radar_z %12.5f \n",  Receive_Data.radar_data[0], Receive_Data.radar_data[1],Receive_Data.radar_data[2]);
    // fprintf(fp_file, "DSO_x %12.5f, DSO_y %12.5f, DSO_z %12.5f \n", Receive_Data.calculate_data[0],Receive_Data.calculate_data[1],Receive_Data.calculate_data[2]);
    // 障碍物距离设定

    float obstacleZmin = 0.1;
    float obstacleZmax = 3;

    // 遍历边缘
    Position initStart;
    Position initEnd;
    Position finalStart;
    Position finalEnd;

    // 发布的 pointcloud message 初始化
    sensor_msgs::PointCloud pointCloudmsg,obtacle_msg;
    pointCloudmsg.header.stamp = ros::Time::now();
    pointCloudmsg.header.frame_id = "map";

    // 障碍物点个数计数
    int numObstacle = 0;
    // 障碍物点vector保存
    std::vector<Index> obstacleSet;
    
    convertCarToMap(Eigen::Vector3f( Car_Width/2.0  + obstacleX, Rear_Axle  + obstacleY, 0), initStart, carMapPose, currR);
    convertCarToMap(Eigen::Vector3f( -Car_Width/2.0 - obstacleX, Rear_Axle  + obstacleY, 0), initEnd, carMapPose, currR);
    convertCarToMap(Eigen::Vector3f( Car_Width/2.0  + obstacleX, 0, 0), finalStart, carMapPose, currR);
    convertCarToMap(Eigen::Vector3f( -Car_Width/2.0 - obstacleX, 0, 0), finalEnd, carMapPose, currR);
// convertCarToMap(Eigen::Vector3f( -0.902 - obstacleX, -1.025 - obstacleY, 0), finalEnd, carMapPose, currR);
    // std::cout << "initStart=" << initStart.transpose() << ", initEnd =" << initEnd.transpose() << ", finalStart =" << finalStart.transpose() <<
        // "finalEnd" << finalEnd.transpose() << std::endl;
    // 车长4.621, 车宽1.804, 前后分别增加1m.
    // 以20cm为一个单位,一共遍历(6.621/0.1)= 66*2 = 132条线;
    // int iterationNum = (int)((4.621+obstacleY*2)/0.1);
    int iterationNum = (int)((Rear_Axle  + obstacleY)/0.1);
    // std::cout << "(4.621+obstacleY*2)" << (4.621+obstacleY*2) << "一共有多少次循环" << iterationNum << std::endl;

    double ll = 0;
    for(int i = 0; i < iterationNum; ++i){
        ll = i;
        Position start = (1 - ll/iterationNum) * initStart + ll/iterationNum * finalStart;
        Position end = (1 - ll/iterationNum) * initEnd + ll/iterationNum * finalEnd;
        Position middle = (start + end) / 2;

        if(i%30 == 0){
            // std::cout << "(1 - i/iterationNum): " << (1 - ll/iterationNum) << "(1 - i/iterationNum) * initStart: " << (1 - ll/iterationNum) * initStart.transpose() << std::endl;
            // std::cout << "i/iterationNum * finalStart: " << ll/iterationNum * finalStart.transpose() << std::endl;
            // std::cout << "第" << i << "次循环时，start=" << start.transpose() << ", end =" << end.transpose() << ", middle =" << middle.transpose() << std::endl;
        }
        
        for (grid_map::LineIterator iterator(*map_, middle, start); !iterator.isPastEnd(); ++iterator) {
            if(map_->at("maxHeight", *iterator) > obstacleZmin){
                numObstacle++;
                obstacleSet.emplace_back((*iterator));
                break;
            }
        }

        for (grid_map::LineIterator iterator(*map_, middle, end); !iterator.isPastEnd(); ++iterator) {
            if(map_->at("maxHeight", *iterator) > obstacleZmin){
                numObstacle++;
                obstacleSet.emplace_back((*iterator));
                break;
            }
        }
    }

    std::cout << "检测到的障碍物点个数" << numObstacle << "size" << obstacleSet.size() << std::endl;

    pointCloudmsg.points.resize(numObstacle);
    obtacle_msg.points.resize(2);
    obtacle_msg.points[0].x = 0;
    obtacle_msg.points[0].y = 0;
    obtacle_msg.points[1].x = 0;
    obtacle_msg.points[1].y = 0;
    printf("enter pubobstaleLine function1\n");
    int pointI = 0;
    if(numObstacle > 0)
    {
	  for (std::vector<Index>::iterator iter=obstacleSet.begin();iter!=obstacleSet.end();++iter){
        printf("enter pubobstaleLine function1\n");
        Position obstacleTmp;
        printf("enter pubobstaleLine function1\n");
        map_->getPosition(*iter, obstacleTmp);
        printf("enter pubobstaleLine function1\n");
        //modify
        pointCloudmsg.points[pointI].x = obstacleTmp[0] - carMapPose[0];
        pointCloudmsg.points[pointI].y = obstacleTmp[1] - carMapPose[1];
        printf("enter pubobstaleLine function1\n");
        std::cout << "obstacleTmp[0]" << obstacleTmp[0] << std::endl;
        std::cout << "obstacleTmp[1]" << obstacleTmp[1] << std::endl;
        std::cout << "carMapPose[0]" <<  carMapPose[0] << std::endl;
        std::cout << "carMapPose[1]" <<  carMapPose[1] << std::endl;
        printf("enter pubobstaleLine function1\n");

        // pointCloudmsg.points[pointI].x = obstacleTmp[0];
        // pointCloudmsg.points[pointI].y = obstacleTmp[1];
        // std::cout << "障碍物点坐标" << obstacleTmp.transpose() << std::endl;
        // std::cout<<"car Map Pose:"<<"x = "<<carMapPose[0]<<"y = "<<carMapPose[1]<<std::endl;

        pointCloudmsg.points[pointI].z = 2.5;
        // if(pointCloudmsg.points[pointI].y > 1.0 || pointCloudmsg.points[pointI].y < -1.0)
            ++pointI;
	  }
    }

    fprintf(fp_file,"size of object:%d\n",pointCloudmsg.points.size());
    //找出离车辆后轴中心最近的障碍物点坐标
    vector<float> left_traje_x,left_traje_y,right_traje_x,right_traje_y;
    for(int i =0;i<pointCloudmsg.points.size();i++)
    {
        if(pointCloudmsg.points[i].y <= -Car_Width/2 && pointCloudmsg.points[i].x >= Rear_Axle){
            left_traje_x.push_back(pointCloudmsg.points[i].y);
            left_traje_y.push_back(pointCloudmsg.points[i].x);
        }
        else if(pointCloudmsg.points[i].y >= Car_Width/2 && pointCloudmsg.points[i].x >= Rear_Axle){
            right_traje_x.push_back(pointCloudmsg.points[i].y);
            right_traje_y.push_back(pointCloudmsg.points[i].x);
        }
        // sendData[0] = (int)(obtacle_msg.points[0].y*1000);
        // sendData[1] = (int)(obtacle_msg.points[0].x*1000);
        // sendData[3] = (int)(obtacle_msg.points[1].y*1000);
        // sendData[4] = (int)(obtacle_msg.points[1].x*1000);
        // sendData[2] = sendData[5] = 0;
    }
    // fprintf(fp_file,"left obtacle x: %f, left obtacle y: %f :: right obtacle x %f, right obtacle y %f\n",obtacle_msg.points[0].x,obtacle_msg.points[0].y,obtacle_msg.points[1].x,obtacle_msg.points[1].y);
    //for(int i = 0;i<left_traje_x.size();i++)
    //    fprintf(fp_file,"left obtacle\t%12.5f,%12.5f\n",left_traje_x[i],left_traje_y[i]);
    //for(int i = 0;i<right_traje_x.size();i++)
    //    fprintf(fp_file,"right obtacle\t%12.5f,%12.5f\n",right_traje_x[i],right_traje_y[i]);
    //fprintf(fp_file,"proper left obtacle\t%12.5f,%12.5f\n", *std::max_element(left_traje_x.begin(),left_traje_x.end()),*std::min_element(left_traje_y.begin(),left_traje_y.end()));
    //fprintf(fp_file,"proper right obtacle\t%12.5f,%12.5f\n", *std::min_element(right_traje_x.begin(),right_traje_x.end()),*std::min_element(right_traje_y.begin(),right_traje_y.end()));
    //pointCloudPub.publish(pointCloudmsg);
    //printf("send Trajectory\n");
}


inline void polygonAdd(GridMap* map, grid_map::Polygon& polygon, const Eigen::Vector3f& carMapPose, const Eigen::Matrix3f& currR, Eigen::Vector3f point){

    // point = rightCamToWorldRd.inverse() * currR * rightCamToWorldRd * point;
    point = rightCamToWorldRd.inverse() * currR * rightCamToWorldRd * point;
    ROS_INFO("point value: %f, %f", point[0], point[1]);
    float pose1 = carMapPose[1] + point[0];
    float pose2 = carMapPose[0] + point[1];
    ROS_INFO("polygon point values: %f, %f", pose1, pose2);
    polygon.addVertex(Position(pose2, pose1));

    Position position(pose2, pose1);
    map->atPosition("maxHeight", position) = 10;

}

inline void convertCarToMap(Eigen::Vector3f point, Position &position, const Eigen::Vector3f& carMapPose, const Eigen::Matrix3f& currR){
    point = rightCamToWorldRd.inverse() * currR * rightCamToWorldRd * point;
    float pose1 = carMapPose[1] + point[0];
    float pose2 = carMapPose[0] + point[1];
    position = Position(pose2, pose1);
}



void frameCb(dso_ros::keypointDSOConstPtr msg, GridMap *map, ros::Publisher& pointCloudPub, std::vector<Eigen::Vector3f>& dsoTraj, 
    ros::Publisher& trajectory_Pub) {

    // 尺度信息
    std::cout<<"receive data:\t"<<msg->coord_y<<std::endl;
    Receive_Data.radar_data[0] = msg->coord_x;
    Receive_Data.radar_data[1] = msg->coord_y;
    Receive_Data.radar_data[2] = msg->coord_z;
    float scale = 0.0;
    scale = msg->scale;
    // Debug
    float maxx=0, minx=0, maxy=0, miny=0;
    // ROS_INFO("Map Info: x value %f, y value %f", map->getLength().x() , map->getLength().y()); // 25 15 

    // 1. 从消息中获取车辆轨迹,记录于dsoTraj
    Eigen::Vector3f camPose;
    camPose << msg->camToWorldx, msg->camToWorldy, msg->camToWorldz;
    camPose *= scale;

    // 坐标转换:从xml中读出的是Tcw, Pw = TwcPc
    Eigen::Vector3f rightCamToWorldT = - rightCamToWorldRd.inverse() * rightCamToWorldTd;
    Eigen::Matrix3f rightCamToWorldR = rightCamToWorldRd.inverse();
    Eigen::Vector3f camWorld = rightCamToWorldR * camPose + rightCamToWorldT;
    // ROS_INFO("CAM POSE IN WORLD: x value %f, y value %f, z value %f", camWorld[0], camWorld[1], camWorld[2]);

    if(usingTrajectory){
        // 调整显示位置
        dsoTraj.emplace_back(Eigen::Vector3f(camWorld[1] - 0.4 * map->getLength().x(), camWorld[0], camWorld[2]));
    }

    // 2. 从消息中获取地图点信息,记录于girdmap中
    for (int i = 0; i < msg->size; ++i){

        // 获取地图点坐标
        Eigen::Vector3f pointWorld((float)msg->x[i],(float)msg->y[i],(float)msg->z[i]);
        pointWorld *= scale;

        // 坐标转换
        pointWorld = rightCamToWorldR * pointWorld + rightCamToWorldT;
        // pointWorld[0] < map->getLength().x() && pointWorld[1] < map->getLength().y() &&

        // 将地图点保存到 grid map中, 剔除高度小于0.2m, 大于3m的点.
        if ( pointWorld[2] > 0.0 && pointWorld[2] < 3 ) {
        // if ( pointWorld[2] > 0.0001 && pointWorld[2] < 100000 ) {
            try {
//              Position position(pointWorld[1] - 0.4 * map->getLength().x(), pointWorld[0] - 0.4 * map->getLength().y());
                Position position(pointWorld[1] - 0.4 * map->getLength().x(), pointWorld[0]);
                // Position position(pointWorld[1],pointWorld[0]);
                map->atPosition("pointNum", position)++;
                // grid map中只记录最高的点,并只保存在一个栅格中个数多于20的数据
                // if (map->atPosition("maxHeight", position) < pointWorld[2] && map->atPosition("pointNum", position) > 0)
                if (map->atPosition("maxHeight", position) < pointWorld[2] && map->atPosition("pointNum", position) > 5)
                { // 去除噪点
                    map->atPosition("maxHeight", position) = pointWorld[2];
                }
            }
            catch (std::out_of_range) {
//                ROS_INFO("x value %f, y value %f, z value %f", pointWorld[0], pointWorld[1], pointWorld[2]);
            }
        }
    }

    // 3. 实时发布当前点云数据
    if(usingPointCloud){
        sensor_msgs::PointCloud pointCloudmsg;
        pointCloudmsg.header.stamp = ros::Time::now();
        pointCloudmsg.header.frame_id = "map";
        pointCloudmsg.points.resize(msg->size);
        for(unsigned int i = 0;i < msg->size; ++i){
            Eigen::Vector3f pointWorld((float)msg->x[i],(float)msg->y[i],(float)msg->z[i]);
            pointWorld *= scale;
            pointWorld = rightCamToWorldR * pointWorld + rightCamToWorldT;
            pointCloudmsg.points[i].x = pointWorld[0];
            pointCloudmsg.points[i].y = pointWorld[1] - 0.4 * map->getLength().x();
            pointCloudmsg.points[i].z = pointWorld[2];
        }
        pointCloudPub.publish(pointCloudmsg);
    }

    // 4. 实时显示当前前相机障碍物点云 
    // TODO: 因为有杂点存在,需要统计每个栅格中点的个数来剔除杂点
    if(usingObstacleDetectFront){

        // 初始化点云消息
        sensor_msgs::PointCloud pointCloudmsg;
        pointCloudmsg.header.stamp = ros::Time::now();
        pointCloudmsg.header.frame_id = "map";
        pointCloudmsg.points.resize(msg->size);
        int debugNum = 0;
        float debugMax = 0;

        // 对每个地图点,计算其与相机位置的距离,要是距离小于20,则将其添加至点云并发布
        for(unsigned int i = 0;i < msg->size; ++i){

            Eigen::Vector3f pointCam((float)msg->x[i],(float)msg->y[i],(float)msg->z[i]);
            pointCam *= scale;
            pointCam = rightCamToWorldR * pointCam + rightCamToWorldT;
            // std::cout << "变化前pointCAM" << pointCam << std::endl;
            // std::cout << "dsotraj最后一维" << camWorld << std::endl;
            // std::cout <<" 变化后 " << pointCam << std::endl;
            
            // 计算距离
            float dist = (pointCam-camWorld).norm();
            if(dist > debugMax)
                debugMax = dist;
            if(dist<20){

                // ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());
                //         for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
                //     Position position;
                //     map.getPosition(*it, position);
                //     map.at("maxHeight", *it) = 0;
                // }
                // Wait for next cycle.

                // 发布轨迹点云消息
                pointCloudmsg.points[i].x = pointCam[1] - 0.4 * map->getLength().x();
                pointCloudmsg.points[i].y = pointCam[0];
                pointCloudmsg.points[i].z = pointCam[2];    
                debugNum++;
            }
        }
        ROS_INFO("debugNum: %d , debugMax: %f\n", debugNum, debugMax);
        pointCloudPub.publish(pointCloudmsg);
    }

    // 显示轨迹
    if(usingTrajectory){

        sensor_msgs::PointCloud pointCloudmsg;
        pointCloudmsg.header.stamp = ros::Time::now();
        pointCloudmsg.header.frame_id = "map";
        pointCloudmsg.points.resize(dsoTraj.size()+5);  // 车辆中心和四个顶点位置
        for(unsigned int i = 0;i < dsoTraj.size(); ++i){
            pointCloudmsg.points[i].x = dsoTraj[i][0];
            pointCloudmsg.points[i].y = dsoTraj[i][1];
            pointCloudmsg.points[i].z = dsoTraj[i][2];
        }

        // 显示车辆后轴中心和顶点坐标
        if(usingCarPose){
            int i = dsoTraj.size();
            Eigen::Vector3f delta = rightCamToWorldTd;
            Eigen::Quaternionf currq(msg->camToWorldRw, msg->camToWorldRx, msg->camToWorldRy, msg->camToWorldRz);
            Eigen::Matrix3f currR = currq.toRotationMatrix();
            delta = rightCamToWorldR * currR * delta;
            pointCloudmsg.points[i].x = pointCloudmsg.points[i-1].x + delta[1];
            pointCloudmsg.points[i].y = pointCloudmsg.points[i-1].y + delta[0];
            pointCloudmsg.points[i].z = pointCloudmsg.points[i-1].z + delta[2]+5;

            Receive_Data.calculate_data[0] = pointCloudmsg.points[i].x;
            Receive_Data.calculate_data[1] = pointCloudmsg.points[i].y;
            Receive_Data.calculate_data[2] = pointCloudmsg.points[i].z;
            std::cout<<"car location:"<<pointCloudmsg.points[i].x<<"\t"<<Receive_Data.calculate_data[0]<<std::endl;
            // 显示顶点坐标
            if(1){
                Eigen::Vector3f tmp;
                tmp << Car_Width/2, Rear_Axle, 0; // 车辆坐标系
                tmp = rightCamToWorldR * currR * rightCamToWorldRd * tmp;
                pointCloudmsg.points[i+1].x = pointCloudmsg.points[i].x + tmp[1];
                pointCloudmsg.points[i+1].y = pointCloudmsg.points[i].y + tmp[0];
                pointCloudmsg.points[i+1].z = 5;

                tmp << -Car_Width/2, Rear_Axle, 0; // 车辆坐标系
                tmp = rightCamToWorldR * currR * rightCamToWorldRd * tmp;
                pointCloudmsg.points[i+2].x = pointCloudmsg.points[i].x + tmp[1];
                pointCloudmsg.points[i+2].y = pointCloudmsg.points[i].y + tmp[0];
                pointCloudmsg.points[i+2].z = 5;

                tmp << Car_Width/2, -(Car_Height - Rear_Axle), 0; // 车辆坐标系
                tmp = rightCamToWorldR * currR * rightCamToWorldRd * tmp;
                pointCloudmsg.points[i+3].x = pointCloudmsg.points[i].x + tmp[1];
                pointCloudmsg.points[i+3].y = pointCloudmsg.points[i].y + tmp[0];
                pointCloudmsg.points[i+3].z = 5;

                tmp << -Car_Width/2, -(Car_Height - Rear_Axle), 0; // 车辆坐标系
                tmp = rightCamToWorldR * currR * rightCamToWorldRd * tmp;
                pointCloudmsg.points[i+4].x = pointCloudmsg.points[i].x + tmp[1];
                pointCloudmsg.points[i+4].y = pointCloudmsg.points[i].y + tmp[0];
                pointCloudmsg.points[i+4].z = 5;
            }
            if(usingObstacleDetect){
                pubGlobalObstacleLine(map, pointCloudPub, Eigen::Vector3f(pointCloudmsg.points[i].x,pointCloudmsg.points[i].y,pointCloudmsg.points[i].z),
                 currR);
            }
        }
        trajectory_Pub.publish(pointCloudmsg);
    }
}

/*
*function : tcp communicate
*add date:2019-03-23
*/
void sysTime(void)
{
    struct timeval tv;
    struct timezone tz;  
    struct tm *t;
    
    gettimeofday(&tv, &tz);
    t = localtime(&tv.tv_sec);
    printf("\ntime_now:%d-%d-%d %d:%d:%d.%ld\n", 1900+t->tm_year, 1+t->tm_mon, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec, tv.tv_usec);
} 


 //add
 void* server(void* args)
 {
	//定义服务器监听套接字和连接套接字
	int listen_fd = -1, connect_fd = -1;//初始化为-1
	struct sockaddr_in servaddr;//定义服务器对应的套接字地址
	//服务器接收和发送缓冲区
	//char sendbuf[MAXLINE], recbuf[MAXLINE];
	 
	//初始化套接字地址结构体
	memset(&servaddr, 0, sizeof(servaddr));
	servaddr.sin_family = AF_INET;//IPv4
	servaddr.sin_port = htons(10004);//设置监听端口
	servaddr.sin_addr.s_addr = htonl(INADDR_ANY);//表示接收任意IP的连接请求
 
	//创建套接字
	if((listen_fd = socket(AF_INET, SOCK_STREAM, 0)) == -1){
		//如果创建套接字失败，返回错误信息
		//strerror(int errnum)获取错误的描述字符串
		printf("create socket error: %s(error: %d)\n", strerror(errno), errno);
		exit(0);
	}


	int on =1;
	int s = setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
	if(s < 0){
		perror("setsocketopt");
		exit(6);
	}
	
    
	//绑定套接字和本地IP地址和端口
	if(bind(listen_fd, (struct sockaddr*)&servaddr, sizeof(servaddr)) == -1){
		//绑定出现错误
		printf("bind socket error: %s(error: %d)\n", strerror(errno), errno);
		exit(0);
	}
 
	//使得listen_fd变成监听描述符
	if(listen(listen_fd, 10) == -1){
		printf("listen socket error: %s(error: %d)\n", strerror(errno), errno);
		exit(0);
	}
 
	//accept阻塞等待客户端请求
	printf("等待客户端发起连接\n");
static bool connect = false;
	while(1){
        if(!connect)
        {
		    if((connect_fd = accept(listen_fd, (struct sockaddr*)NULL, NULL)) == -1){
			    printf("accept socket error: %s(error: %d)\n", strerror(errno), errno);
			    continue;
		    }
            connect = true;
        }


 
		//可以一直保持连接
		//while(1){
			//读取客户端发来的信息
			//ssize_t len = read(connect_fd, recbuf, sizeof(recbuf));
			//if(len < 0){
			//	if(errno == EINTR){
			//		continue;
			//	}
			//	exit(0);
			//}
 
			//printf("接收客户端的请求：%s\n", recbuf);
 
			//向客户端发送信息
			usleep(20000);
			printf("向客户端发送信息：\n");
			for(int i = 0; i < 6; ++i)
 			{
                //  sendData[i]+=1;
 				printf("%d\t",sendData[i]);
 			}
			//fgets(sendbuf, sizeof(sendbuf), stdin);
			write(connect_fd, (int *) sendData, 6*sizeof(int));
			sysTime();
			
		//}
 
		//关闭连接套接字
		// close(connect_fd);
		// break;
	}
	//关闭监听套接字
	// close(listen_fd);
}
