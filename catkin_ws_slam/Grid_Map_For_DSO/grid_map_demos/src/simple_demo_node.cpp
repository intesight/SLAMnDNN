/*
 * simple_demo_node.cpp
 *
 *  Created on: Nov 01, 2018
 *      Author: Mario Ning
 *   Institute: Intesight Co.
 */

#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include "grid_map_msgs/keyframeMsg.h"
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <stdlib.h>
#include <sophus/sim3.hpp>
#include <sophus/se3.hpp>

using namespace grid_map;

struct InputPointDense {
    float idepth;
    float idepth_var;
    uchar color[4];
};

/**
 * 将msg信息转换成 Grid Map
 * @param msg
 * @param map
 */
void frameCb(grid_map_msgs::keyframeMsgConstPtr msg, GridMap *map) {

    // ROS_INFO("get msg from lsd_slam 1 [%f]", map->getLength().x());

    int height = msg->height;
    int width = msg->width;
    float fxi = 0.008586;
    float fyi = 0.004966;
    float cxi = -2.747478;
    float cyi = -1.589004;

    InputPointDense *originalInput = new InputPointDense[width * height];
    memcpy(originalInput, msg->pointcloud.data(), width * height * sizeof(InputPointDense));

    Sophus::Sim3f camToWorld = Sophus::Sim3f();
    memcpy(camToWorld.data(), msg->camToWorld.data(), 7 * sizeof(float));

    float lamda = 1.68;

    float my_scale = camToWorld.scale();

    for (int y = 1; y < height - 1; ++y)
        for (int x = 1; x < width - 1; ++x) {

            // 深度无效,继续
            if (originalInput[x + y * width].idepth <= 0) continue;
            float depth = 1 / originalInput[x + y * width].idepth;
            float depth4 = depth * depth;
            depth4 *= depth4;
            if (originalInput[x + y * width].idepth_var * depth4 > 0.01)
                continue;
            if (originalInput[x + y * width].idepth_var * depth4 * my_scale * my_scale > 0.1)
                continue;

            // 将像素点转换到相机坐标
            float tmp_point[3];
            tmp_point[0] = (x * fxi + cxi) * depth;
            tmp_point[1] = (y * fyi + cyi) * depth;
            tmp_point[2] = depth;

            // 相机坐标系转换到世界坐标系
            float pointcloud_x = tmp_point[0];
            float pointcloud_y = tmp_point[1];
            float pointcloud_z = tmp_point[2];
            float transform_matrix[] = {-0.076236128, -0.997012510, -0.012414047, 2012.619045345, -0.671809121,
                                        0.060561654, -0.738244398, 1211.690885156, 0.736790715, -0.047941025,
                                        -0.674419083, 15.044303882};
            Eigen::Matrix<float, 3, 3> Rotation_vehicle2cam;
            Eigen::Matrix<float, 3, 3> iRotation_vehicle2cam;
            Eigen::Vector3f Translation_vehicle2cam;
            Eigen::Vector3f dest_point;
            //	Eigen::Vector3f pointcloud{pointcloud_x,pointcloud_y,pointcloud_z};
            Rotation_vehicle2cam << transform_matrix[0], transform_matrix[1], transform_matrix[2],
                    transform_matrix[4], transform_matrix[5], transform_matrix[6],
                    transform_matrix[8], transform_matrix[9], transform_matrix[10];
            iRotation_vehicle2cam = Rotation_vehicle2cam.inverse();
            Translation_vehicle2cam << transform_matrix[3], transform_matrix[7], transform_matrix[11];
            Sophus::Vector3f pointcloud = camToWorld * Sophus::Vector3f(pointcloud_x, pointcloud_y, pointcloud_z);
            Translation_vehicle2cam(0) = lamda * pointcloud[0] - Translation_vehicle2cam(0) / 1000;
            Translation_vehicle2cam(1) = lamda * pointcloud[1] - Translation_vehicle2cam(1) / 1000;
            Translation_vehicle2cam(2) = lamda * pointcloud[2] - Translation_vehicle2cam(2) / 1000;
            dest_point = iRotation_vehicle2cam * Translation_vehicle2cam; // In car coordinate [m]

            // ROS_INFO("x value %f, y value %f", dest_point[0], dest_point[1]);
            // ROS_INFO("x value %f, y value %f", map->getLength().x() , map->getLength().y());
            if (dest_point[0] < map->getLength().x() && dest_point[1] < map->getLength().y() && dest_point[2] < 3 &&
                dest_point[2] > 0.1) {
                try {
                    Position position(dest_point[1] - 0.4 * map->getLength().x(),
                                      dest_point[0] - 0.4 * map->getLength().y());
                    map->atPosition("num", position)++;
                    if (map->atPosition("elevation", position) < dest_point[2] &&
                        map->atPosition("num", position) > 100) { // 去除噪点
                        map->atPosition("elevation", position) = dest_point[2];
                        map->atPosition("OccupancyMap", position) = 1;
                    }
                }
                catch (std::out_of_range) {
                    ROS_INFO("x value %f, y value %f, z value %f", dest_point[0], dest_point[1], dest_point[2]);
                }
            }

            //   if(tmp_point[0] < map->getLength().x() && tmp_point[1] < map->getLength().y() && tmp_point[2]<3 && tmp_point[2]>0.1){
            //   try
            //   {
            //     Position position(tmp_point[0],tmp_point[1]);
            //     map->atPosition("elevation",position) = tmp_point[2];              }
            //   catch (std::out_of_range)
            //   {
            //     ROS_INFO("x value %f, y value %f, z value %f", tmp_point[0], tmp_point[1], tmp_point[2]);
            //   }
            // }

        }

    float sum = 0;
    int mapWidth = map->getSize()(1);
    int mapHeight = map->getSize()(0);

    Index submapStartIndex;
    Index submapBufferSize;
    Index indexFirst;

    // 第一列记录这一行有多少个点有效,试图得到车位点信息
    for (int i = 0; i < mapHeight; i++) {

        sum = 0;
        submapStartIndex << i, (mapWidth / 2);
        submapBufferSize << 1, (mapWidth / 2);

        for (grid_map::SubmapIterator iterator(*map, submapStartIndex, submapBufferSize);
             !iterator.isPastEnd(); ++iterator) {
            // ROS_INFO("value%f", map->at("elevation", *iterator));

            if (map->at("elevation", *iterator) > 0.8) {
                sum++;
                // sum = sum + map->at("elevation", *iterator);
                // ROS_INFO("Output");
            }
        }

        indexFirst << i, 0;
        map->at("elevation", indexFirst) = sum / mapWidth * 20;
        // ROS_INFO("First value %f", sum / sumId);
    }

    // int i = 0;
    // int sum = 0;
    // int sumId = 0;

    // for (grid_map::GridMapIterator iterator(*map); !iterator.isPastEnd(); ++iterator) {

    //     ++i;

    //     if(map->at("elevation", *iterator) > 0.001){
    //         sum += map->at("elevation", *iterator);
    //         ++sumId;
    //     }
    //     if(i == map->getLength().y()*10){
    //         if(sumId != 0)
    //             map->at("elevation", *iterator) = sum / sumId;
    //         else{
    //             sumId = 0;
    //             sum = 0;
    //             i = 0;
    //         }
    //     }
    // }

    delete[] originalInput;
}

int main(int argc, char **argv) {

    // Create grid map.
    GridMap map({"elevation", "num", "OccupancyMap", "Trajectory"});
    map.setFrameId("map");
    map.setGeometry(Length(25, 15), 0.1);   // 0.1m resolution

    for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
        Position position;
        map.getPosition(*it, position);
        map.at("elevation", *it) = 0;
        map.at("num", *it) = 0;
        map.at("OccupancyMap", *it) = -1;
        map.at("Trajectory", *it) = -1;
    }

    // Initialize node and publisher.
    ros::init(argc, argv, "grid_map_simple_demo");
    ros::NodeHandle nh("~");
    ros::Publisher publisher = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
    ros::Subscriber keyFrames_sub = nh.subscribe<grid_map_msgs::keyframeMsg>("lsd_slam/keyframes", 20,
                                                                             boost::bind(&frameCb, _1, &map));

    // Work with grid map in a loop.
    ros::Rate rate(30.0);   // 30hz

    while (nh.ok()) {

        // Publish grid map.
        ros::Time time = ros::Time::now();
        map.setTimestamp(time.toNSec());
        grid_map_msgs::GridMap message;

        GridMapRosConverter::toMessage(map, message);
        publisher.publish(message);

        // ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());
        //         for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
        //     Position position;
        //     map.getPosition(*it, position);
        //     map.at("elevation", *it) = 0;
        // }
        // Wait for next cycle.

        ros::spinOnce();

        rate.sleep();

    }

    return 0;
}
