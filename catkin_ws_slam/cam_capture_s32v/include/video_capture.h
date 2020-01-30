/*
 * video_capture.h
 *
 *  Created on: Dec 24, 2015
 *      Author: Jim
 */

#ifndef VIDEO_CAPTURE_H_
#define VIDEO_CAPTURE_H_

#include"ImageStitching.h"
#include <linux/videodev2.h>

typedef unsigned long  int UInt64_t;
typedef unsigned int UInt32_t;
typedef  int Int32_t;
typedef signed short		Int16_t;
typedef  unsigned char      uchar;
extern unsigned char mem_video_frontyuyv[1280*720*2];
extern unsigned char mem_video_backyuyv[1280*720*2];
extern unsigned char mem_video_leftyuyv[1280*720*2];
extern unsigned char mem_video_rightyuyv[1280*720*2];
extern unsigned char mem_video_frontuyvy[1280*720*2];
extern unsigned char mem_video_backuyvy[1280*720*2];
extern unsigned char mem_video_leftuyvy[1280*720*2];
extern unsigned char mem_video_rightuyvy[1280*720*2];

/* check the supported webcam resolutions using $v4l2-ctl --list-formats-ext */
#define IM_WIDTH   1920
#define IM_HEIGHT 1080
#define CLEAR(x) memset (&(x), 0, sizeof (x))

struct buffer {
	void * start;
	size_t length;
};





typedef struct CvRect1
{
    int x;
    int y;
    int width;
    int height;
}
        CvRect1;

CvRect1 cvRect1(int init_x, int init_y, int rect_width, int rect_height);

/******************************************************************************************************/

//unsigned char  front_pjsdx[1280 * 720 * 2];

//unsigned char  front_puyvy[IMG_WIDTH * IMG_HEIGHT * 2];
//unsigned char  back_puyvy[IMG_WIDTH * IMG_HEIGHT * 2];
//unsigned char  left_puyvy[IMG_WIDTH * IMG_HEIGHT * 2];
//unsigned char  right_puyvy[IMG_WIDTH * IMG_HEIGHT * 2];
//
//unsigned char  front_pyuyv[IMG_WIDTH * IMG_HEIGHT * 2];
//unsigned char  back_pyuyv[IMG_WIDTH * IMG_HEIGHT * 2];
//unsigned char  left_pyuyv[IMG_WIDTH * IMG_HEIGHT * 2];
//unsigned char  right_pyuyv[IMG_WIDTH * IMG_HEIGHT * 2];
//int   size_img  = IMG_WIDTH * IMG_HEIGHT * 2;
void ReadOrigialmage(void);
void ReadLut(void);
int  saveframe(char *str, void *p, int length, int is_oneframe);

void rotate270(unsigned char *src,unsigned char *dst,int width,int height);
/******************************************************************************************************/
typedef struct CvPoint1
{
    int x;
    int y;
}
        CvPoint1;
//CvPoint1  car_up_left, car_down_right;

typedef struct TacPoint_Struct
{
    Int16_t x;
    Int16_t y;
}TacPoint;

typedef struct TabBev_Struct
{
    TacPoint point_pos;
    Int32_t wt_fusion;
    Int32_t wt_upleft;
    Int32_t wt_upright;
    Int32_t wt_downleft;
    Int32_t wt_downright;
}TabBev;




///TabBev **bev_Table[4];
/**************************************************************************************
 * @param result_image is stitching output ;
 * @param front_image_uyvy  source pic
 * @param back_image_uyvy source pic
 * @param left_image_uyvy  source pic
 * @param right_image_uyvy  source pic
 * @param car_up_left car log position
 * @param car_down_right  log position
 * @param frontback_fov_height  stitching picture height
 * @param leftright_fov_width  stitching picture left and right
 * @param result_width
 * @param source_width
 * @param bev_Table
 * @return
 ********************************************************************************************/
Int32_t bev_process(
        uchar* result_image,
        uchar* front_image_uyvy,
        uchar* back_image_uyvy,
        uchar* left_image_uyvy,
        uchar* right_image_uyvy,
        CvPoint1 car_up_left,
        CvPoint1 car_down_right,
        Int32_t frontback_fov_height,
        Int32_t leftright_fov_width,
        Int32_t result_width,
        Int32_t source_width,
        TabBev **bev_Table[4]);

void analysis_fusion_region_lut_uyvy(uchar* result_image,
                                     uchar* p_src1,
                                     uchar* p_src2,
                                     Int32_t fusion_pic1,
                                     Int32_t fusion_pic2,
                                     CvRect1* region_roi,
                                     Int32_t result_widthstep,
                                     Int32_t src_widthstep,
                                     TabBev **bev_Table[4]);
void analysis_single_region_lut_uyvy(
        uchar* result_image, uchar* p_src,
        CvRect1* region_roi,
        Int32_t camid,
        Int32_t result_widthstep,
        Int32_t src_widthstep,
        TabBev **bev_Table[4]);



Int32_t Bev_Tab_Init(UInt32_t* p_pos_fusion_lut, UInt32_t* p_wt_lut,
                     Int32_t camid, Int32_t fov_width,
                     Int32_t fov_height, TabBev **bev_Table[4]);

Int32_t  load_config_file(const char* base_path, CvPoint1& car_up_left ,CvPoint1& car_down_right,
                          Int32_t& front_fov_height,  Int32_t& back_fov_height,
                          Int32_t& left_fov_width,Int32_t& right_fov_width);
void uyvy_to_yuyv(uchar* pYuyv, uchar* pUyvy, Int32_t width, Int32_t height);
void yuyv_to_uyvy(uchar* pUyvy, uchar* pYuyv, Int32_t width, Int32_t height);
void init_video_capture(char* dev);
void free_video_capture();
void video_capture_streams(unsigned char* mem_video_front, unsigned char* mem_video_back, unsigned char* mem_video_left, unsigned char* mem_video_right);

#endif /* VIDEO_CAPTURE_H_ */
