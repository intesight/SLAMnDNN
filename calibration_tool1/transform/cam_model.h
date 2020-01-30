//
// Created by chengguoqiang on 18-5-16.
//

#ifndef CALIB_MODEL_LUT_CAM_MODEL_H
#define CALIB_MODEL_LUT_CAM_MODEL_H

#include "common_parkinggo.h"
#include <stdio.h>
typedef struct cam_model_struct
{
    double fx;
    double cx;
    double fy;
    double cy;
    double k1;
    double k2;
    double k3;
    double k4;
} cam_model_s;

int cam2pixel(IN cam_model_s cam_model,float cam_world_point[3], INOUT float pixel[2]);
//int pixel2cam(IN FILE* fpWrite_x, IN FILE*fpWrite_y,IN cam_model_s cam_model, INOUT float cam_world_point[3], IN float pixel[2]);
int pixel2cam(IN cam_model_s cam_model, INOUT float cam_world_point[3], IN float pixel[2]);

#endif //CALIB_MODEL_LUT_CAM_MODEL_H
