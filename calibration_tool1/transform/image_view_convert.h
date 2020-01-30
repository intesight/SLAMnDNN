//
// Created by chengguoqiang on 18-5-16.
//

#ifndef CALIB_MODEL_LUT_IMAGE_VIEW_CONVERT_H
#define CALIB_MODEL_LUT_IMAGE_VIEW_CONVERT_H

//#define PANORAMA_UYVY


#include "common_parkinggo.h"
#include <iostream>

#include <opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include "cam_model.h"
using namespace cv;

//extern float yCoef;

typedef signed int			Int32_t;

typedef enum image_view_enum
{
    image_view_min = 0,
    image_view_birdview_e, //俯视图
    image_view_projective_e, //透视图
    image_view_stitch_e,   //俯视拼接融合图
    image_view_multiview_e, //多视角透视展开图
    image_view_max,
}IMAGE_VIEW_E;

typedef enum view_enum
{
    view_min = 0,
    view_front_e,
    view_rear_e,
    view_left_e,
    view_right_e,
    view_max,
}VIEW_E;

//配合image_view_stitch_e使用，其它情况无效
typedef enum fusion_enum
{
    fusion_min = 0,
    fusion_front_e, //前区域需要融合，后区域不需要融合 ,针对左、右相机情况下
    fusion_rear_e, //后区域需要融合，前区域不需要融合,针对左、右相机情况下
    fusion_left_e, //左区域需要融合，右区域不需要融合,针对前、后相机情况下
    fusion_right_e, //右区域需要融合，左区域不需要融合,针对前、后相机情况下
    fusion_left_right_e, //左、右同时需要融合,针对前、后相机情况下
    fusion_front_rear_e, //前、后同时需要融合,针对左、右相机情况下
    fusion_max,
}FUSION_E;

//像素点从一透视图转换到另一透视图的像素点,注意xml文件中的offset必须为透视图2的offset
int projectivepixel2projectivepixel(IN VIEW_E view_index,IN float projective_1_pixel[2], IN float projective_1_matirx[3][3], IN float projective2_fov[2], IN float projective2_size[2], OUT float projective_2_pixel[2]);
//获取参数配置文件
int get_parameter(std::string file_name);

//生成去畸变平面透视图
int undistort_plane_image(IN Mat raw_image, INOUT Mat undistort_image, IN float fov[2],IN VIEW_E view_index);

//柱面展开
int undistort_cylinder_image(IN Mat raw_image, INOUT Mat column_image, IN float fov[2],IN VIEW_E view_index);
//生成多视角去畸变平面透视图
int multiview_undistort_plane_image(IN Mat raw_image, OUT Mat undistort_image, IN float fov[2],IN VIEW_E view_index);

//生成俯视图
Mat birdview_image_generate(IN Mat raw_image, IN VIEW_E view_index, IN bool stitch_select_b, IN float stretch_coef, IN float yCoef);

//使用lut table，生成相应的效果图
int lutTable_generate_image(IN Mat raw_image, INOUT Mat dst_image, IN IMAGE_VIEW_E image_view_e, IN VIEW_E view_index, IN FUSION_E stitch_fusion);

//生成一张包含前、后、左、右相机俯视图的非融合拼接图
Mat birdview_image_stitch(IN Mat front_image, IN Mat rear_image, IN Mat left_image, IN Mat right_image);

//已知某一原图上的点坐标，求出此点对应的透视图上的坐标
int rawpixel2projective(IN float raw_pixel[2], IN VIEW_E view_index, IN float projective_fov[2],IN float projective_size[2], OUT float projective_pixel[2]);

//已知某一俯视图上的点坐标，求出此点对应的原图图上的坐标
int birdviewpoint2rawpixel(IN float stitchpoint[2], IN VIEW_E view_index, INOUT float rawpixel[2]);

//世界坐标系下的4个物理点，转换为透视图上的点
int worldview2undistortpixel(IN VIEW_E view_index,  IN float projective_fov[2],IN float projective_size[2]);

//俯视图上的像素到世界物理坐标的转换
int birdviewpix2worldpoint(IN float stitchpoint[2], IN VIEW_E view_index, INOUT float worldpoint[3]);

//使用lut table，生成俯视融合拼接图
Mat lutTable_generate_stitchFusion(IN Mat front_raw_image, IN Mat rear_raw_image, IN Mat left_raw_image, IN Mat right_raw_image, IN bool front_use, IN bool rear_use, IN bool left_use, IN bool right_use);

// liuli[12/18]
Int32_t generate_stitchFusion_yuv(
        uchar* result_image,
        uchar* front_image_uyvy,
        uchar* back_image_uyvy,
        uchar* left_image_uyvy,
        uchar* right_image_uyvy, IN bool front_use, IN bool rear_use, IN bool left_use, IN bool right_use);
//bgr to yuv
void bgrToYuv(float bgr[3], float yuv[3]);

//yuv to bgr
void yuvToBgr(float yuv[3], float bgr[3]);

//输入融合单路图，bgr2yuv + gain + yuv2bgr 输出加上增益后到图
int image_addGain(INOUT Mat stitch_raw_image, IN VIEW_E view_index);

//根据四张原始鱼眼图，求解增益
int generate_gain(float varN,  float varG, IN Mat front_raw_image, IN Mat rear_raw_image, IN Mat left_raw_image, IN Mat right_raw_image);

//透视矩阵
void undistort_generate_matirx(IN int matrix_rowoffset, IN int matrix_coloffset, IN float fov[2], IN int dst_image_size[2], INOUT float matrix[3][3], IN VIEW_E view_index);

//int pix2cam_model(IN FILE* fpWrite_x, IN FILE*fpWrite_y, IN VIEW_E view_index, INOUT float cam_world_point[3], IN float raw_pixel[2]);
int pix2cam_model( IN VIEW_E view_index, INOUT float cam_world_point[3], IN float raw_pixel[2]);

//获取畸变系数
double get_polycoeff(double k[4]);

//拼接俯视图像素转透视图坐标
int birdviewpix2projectpoint(IN float stitchpoint[2], INOUT float projective_point[2],  IN VIEW_E view_index, IN float projective_fov[2], IN float projective_size[2]);

//函数功能： 给定原俯视图某一点的像素坐标(u,v)，输出对应拉伸后的俯视图像素坐标(u',v')。
//IN： float stitchpoint[2] 原俯视图下像素坐标，坐标,[0] u坐标; [1] v坐标
//INOUT： float modifiedpoint[2] 拉伸后平行四边形像素坐标 [0] u‘坐标; [1] v’坐标
void birdview2birdview(IN float stitchpoint[2], INOUT float modifiedpoint[2], IN float stretch_coef, IN float yCoef);

//由角度获取拉伸系数
//float get_stretch_coef(IN float cord[6], IN float alpha, IN float yCoef);
float get_stretch_coef(IN float alpha, IN float theta, IN float yCoef);

//给定车位与水平方向的夹角theta,选择合适的lut表并返回所对应的拉伸系数stretch_coef,yCoef
int get_stretchParms(std::string file_name, float theta, INOUT float &stretch_coef, INOUT float &yCoef);

/* function: 给定车位与水平方向的夹角theta,得到拉伸后的斜车位
* parametes:
*          IN： Mat raw_image,原始四路鱼眼图
*          IN： float theta，原车位与水平方向的夹角
*          INOUT： Mat stitch_fusion, 变换后的斜车位拼接图
*          stretch_coef ,yCoef 斜车位对应的拉伸系数 */
void stitch_thetaLot(IN Mat raw_image, INOUT Mat &stitch_fusion, IN float theta, INOUT float &stretch_coef, INOUT float &yCoef);
#endif //CALIB_MODEL_LUT_IMAGE_VIEW_CONVERT_H


