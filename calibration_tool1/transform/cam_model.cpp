//
// Created by chengguoqiang on 18-5-16.
//

#include "cam_model.h"
#include <math.h>
#include "image_view_convert.h"
/*
 * author: cheng
 * date:
 * name:cam2pixel
 * function:相机坐标系到像素坐标系的转换
 * parametes:
 *          IN :cam_model_s cam_model 相机内参
 *          OUT：float cam_world_point[3] 相机坐标系下的物理点
 *        INOUT：float pixel[2] 原始图上额像素坐标 [0] 行; [1] 列
 *
 *return;
 * */
int cam2pixel(IN cam_model_s cam_model, IN float cam_world_point[3], INOUT float pixel[2])
{
    int rtn = 0;
    float xc = cam_world_point[0];
    float yc = cam_world_point[1];
    float zc = cam_world_point[2];

    double fx = cam_model.fx;
    double fy = cam_model.fy;
    double cx = cam_model.cx;
    double cy = cam_model.cy;
    double k1 = cam_model.k1;
    double k2 = cam_model.k2;
    double k3 = cam_model.k3;
    double k4 = cam_model.k4;
    // https://docs.opencv.org/master/db/d58/group__calib3d__fisheye.html
    float a = xc/zc;      //归一化坐标（迪卡尔坐标系）
    float b = yc/zc;
    float r = sqrt(a*a + b*b);   //极坐标系
    float theta = atan(r);       //极角

    float theta_d = theta * (1 + k1*theta*theta + k2*theta*theta*theta*theta + k3*theta*theta*theta*theta*theta*theta + k4*theta*theta*theta*theta*theta*theta*theta*theta);

    float xi = a * theta_d/r;
    float yi = b * theta_d/r;

    if (r > ZERO_LOWER_LIMIT && r < ZERO_UPPER_LIMIT)
    {
        xi = 0;
        yi = 0;
    }

    float u = fx*xi + cy;
    float v = fy*yi + cx;

    u = u < 1 ? 0:u;
    v = v < 1 ? 0:v;

    pixel[0] = u;
    pixel[1] = v;

    return rtn;

}

/*
 * author: cheng
 * date:
 * name:pixel2cam
 * function:像素坐标系到相机坐标系的转换
 * parametes:
 *          IN :cam_model_s cam_model 相机内参
 *          INOUT：float cam_world_point[3] 相机坐标系下的物理点
 *          IN：float pixel[2] 原始图上像素坐标,[0] 行; [1] 列
 *
 *return;
 * */
int pixel2cam(IN cam_model_s cam_model, INOUT float cam_world_point[3], IN float pixel[2])
{
    int rtn = 0;

//    double k[4];
//    get_polycoeff(k);
//    double k1 = k[1];
//    double k2 = k[2];
//    double k3 = k[3];
//    double k4 = k[4];
    double fx = cam_model.fx;
    double fy = cam_model.fy;
    double cx = cam_model.cx;
    double cy = cam_model.cy;
    double k1 = cam_model.k1;
    double k2 = cam_model.k2;
    double k3 = cam_model.k3;
    double k4 = cam_model.k4;

    float v = pixel[0];
    float u = pixel[1];

    float uu = u - cx;
    float vv = v - cy;

    double theat_d = sqrt(uu * uu + vv * vv)/fx;
    double coeff[10] = {-theat_d,1,0,k1,0,k2,0,k3,0,k4};

    cv::Mat coeffs(1,10,CV_64FC1,coeff);
    cv::Mat roots;
    int maxIters = 1000;

    cv::solvePoly(coeffs,roots, maxIters);

    double theat = roots.at<double>(0.0);

//    double theat = 0;    //matlab polyfit()求解的系数
//    theat = theat_d + k1* theat_d* theat_d* theat_d + k2* theat_d* theat_d* theat_d* theat_d* theat_d + k3* theat_d* theat_d* theat_d* theat_d* theat_d* theat_d* theat_d + k4* theat_d* theat_d* theat_d* theat_d* theat_d* theat_d* theat_d* theat_d* theat_d;

    double radius = tan(theat);

    double xi = (u - cx)/fx;
    double yi = (v - cy)/fy;

    float a = xi * radius/theat_d;
    float b = yi * radius/theat_d;

    cam_world_point[0] = a;
    cam_world_point[1] = b;
    cam_world_point[2] = 1.0;

    return rtn;

}