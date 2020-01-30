// Intesight:
// Created by chengguoqiang on 18-5-16.


#define __test_intri__ 1

#include "image_view_convert.h"

#include <iostream>
#include <math.h>

#include<opencv2/imgproc/imgproc.hpp>

using  namespace std;
using  namespace cv;

FileStorage fs; //Opencv 读写操作

int xml_version = 0;

int B;
int G;
int R;

double poly_k1;
double poly_k2;
double poly_k3;
double poly_k4;

bool DEBUG_bilinear;     //xml  是否双线性插值

int matrix_rowoffset;
int matrix_coloffset;


int front_matrix_rowoffset;
int front_matrix_coloffset;

int rear_matrix_rowoffset;
int rear_matrix_coloffset;

int left_matrix_rowoffset;
int left_matrix_coloffset;

int right_matrix_rowoffset;
int right_matrix_coloffset;



int stitch_width;
int stitch_height;

int front_world_view;
int rear_world_view;
int left_world_view;
int right_world_view;

int car_world_width;
int car_world_height;
int car_axle_coord;

int left_pixel_height;    //定义成全局

//透视标注用物理范围框,具体数值以相机为坐标原点,相机离坐标原点，车辆后轴中心在y轴上的距离
int right_cam_height_y;
int  left_cam_height_y;

//front 以相机位置位坐标原点，光心往外为y正方向,相机往右前方向为x正方向
int front_projective_lefttop_x;
int front_projective_lefttop_y;
int front_projective_leftbut_x;
int front_projective_leftbut_y;
int front_projective_righttop_x;
int front_projective_righttop_y;
int front_projective_rightbut_x;
int front_projective_rightbut_y;

//rear 以相机位置位坐标原点，光心往外为y正方向,相机往右前方向为x正方向
int rear_projective_lefttop_x;
int rear_projective_lefttop_y;
int rear_projective_leftbut_x;
int rear_projective_leftbut_y;
int rear_projective_righttop_x;
int rear_projective_righttop_y;
int rear_projective_rightbut_x;
int rear_projective_rightbut_y;

//left 以相机位置位坐标原点，光心往外为x正方向,相机往车前方向为y正方向
int left_projective_lefttop_x;
int left_projective_lefttop_y;
int left_projective_leftbut_x;
int left_projective_leftbut_y;
int left_projective_righttop_x;
int left_projective_righttop_y;
int left_projective_rightbut_x;
int left_projective_rightbut_y;

//right 以相机位置坐标原点，光心往外为x正方向,相机往车前方向为y正方向
int right_projective_lefttop_x;
int right_projective_lefttop_y;
int right_projective_leftbut_x;
int right_projective_leftbut_y;
int right_projective_righttop_x;
int right_projective_righttop_y;
int right_projective_rightbut_x;
int right_projective_rightbut_y;

static  string run_dir;


int front_height = 0;
int front_width = 0;
int rear_height = 0;
int rear_width = 0;
int left_height = 0;
int left_width = 0;
int right_width = 0;
int right_height = 0;

FILE* fp_lut_front;
FILE* fp_lut_rear;
FILE* fp_lut_left;
FILE* fp_lut_right;

FILE* fp_fusion_front;
FILE* fp_fusion_rear;
FILE* fp_fusion_left;
FILE* fp_fusion_right;


uint32_t *file_data_front = NULL;
uint32_t *file_data_rear = NULL;
uint32_t *file_data_left = NULL;
uint32_t *file_data_right = NULL;

float *file_fusion_front = NULL;
float *file_fusion_rear = NULL;
float *file_fusion_left = NULL;
float *file_fusion_right = NULL;
void bgrToYuv(float bgr[3], float yuv[3]);
void yuvToBgr(float yuv[3], float bgr[3]);

int currentGains[12];


int get_parameter(string file_name)
{
    fs.open(file_name, FileStorage::READ);
    if (!fs.isOpened())
    {
        cout<<"open xml file ""failed"<< file_name << "failed" << endl;
        return -1;
    }


    fs["version_name"] >> xml_version;

    fs["run_directory"]>>run_dir;

    fs["B"] >> B;
    fs["G"] >> G;
    fs["R"] >> R;

//    fs["stretch_coef"] >> stretch_coef;
    fs["DEBUG_bilinear"] >> DEBUG_bilinear;

    fs["poly_k1"] >> poly_k1;
    fs["poly_k2"] >> poly_k2;
    fs["poly_k3"] >> poly_k3;
    fs["poly_k4"] >> poly_k4;

    fs["front_matrix_rowoffset"] >> front_matrix_rowoffset;
    fs["front_matrix_coloffset"] >> front_matrix_coloffset;


    fs["rear_matrix_rowoffset"] >> rear_matrix_rowoffset;
    fs["rear_matrix_coloffset"] >> rear_matrix_coloffset;

    fs["left_matrix_rowoffset"] >> left_matrix_rowoffset;
    fs["left_matrix_coloffset"] >> left_matrix_coloffset;

    fs["right_matrix_rowoffset"] >> right_matrix_rowoffset;
    fs["right_matrix_coloffset"] >> right_matrix_coloffset;

    fs["front_word_view"] >> front_world_view;
    fs["rear_world_view"] >> rear_world_view;
    fs["left_world_view"] >> left_world_view;
    fs["right_world_view"] >> right_world_view;

    fs["car_world_width"] >> car_world_width;
    fs["car_world_height"] >> car_world_height;
    fs["car_axle_coord"] >> car_axle_coord;

    fs["left_pixel_height"] >> left_pixel_height;

    fs["right_cam_height_y"] >> right_cam_height_y;
    fs["left_cam_height_y"] >> left_cam_height_y;

    fs["front_projective_lefttop_x"] >> front_projective_lefttop_x;
    fs["front_projective_lefttop_y"] >> front_projective_lefttop_y;
    fs["front_projective_leftbut_x"] >> front_projective_leftbut_x;
    fs["front_projective_leftbut_y"] >> front_projective_leftbut_y;
    fs["front_projective_righttop_x"] >> front_projective_righttop_x;
    fs["front_projective_righttop_y"] >> front_projective_righttop_y;
    fs["front_projective_rightbut_x"] >> front_projective_rightbut_x;
    fs["front_projective_rightbut_y"] >> front_projective_rightbut_y;

    fs["rear_projective_lefttop_x"] >> rear_projective_lefttop_x;
    fs["rear_projective_lefttop_y"] >> rear_projective_lefttop_y;
    fs["rear_projective_leftbut_x"] >> rear_projective_leftbut_x;
    fs["rear_projective_leftbut_y"] >> rear_projective_leftbut_y;
    fs["rear_projective_righttop_x"] >> rear_projective_righttop_x;
    fs["rear_projective_righttop_y"] >> rear_projective_righttop_y;
    fs["rear_projective_rightbut_x"] >> rear_projective_rightbut_x;
    fs["rear_projective_rightbut_y"] >> rear_projective_rightbut_y;

    fs["left_projective_lefttop_x"] >> left_projective_lefttop_x;
    fs["left_projective_lefttop_y"] >> left_projective_lefttop_y;
    fs["left_projective_leftbut_x"] >> left_projective_leftbut_x;
    fs["left_projective_leftbut_y"] >> left_projective_leftbut_y;
    fs["left_projective_righttop_x"] >> left_projective_righttop_x;
    fs["left_projective_righttop_y"] >> left_projective_righttop_y;
    fs["left_projective_rightbut_x"] >> left_projective_rightbut_x;
    fs["left_projective_rightbut_y"] >> left_projective_rightbut_y;

    fs["right_projective_lefttop_x"] >> right_projective_lefttop_x;
    fs["right_projective_lefttop_y"] >> right_projective_lefttop_y;
    fs["right_projective_leftbut_x"] >> right_projective_leftbut_x;
    fs["right_projective_leftbut_y"] >> right_projective_leftbut_y;
    fs["right_projective_righttop_x"] >> right_projective_righttop_x;
    fs["right_projective_righttop_y"] >> right_projective_righttop_y;
    fs["right_projective_rightbut_x"] >> right_projective_rightbut_x;
    fs["right_projective_rightbut_y"] >> right_projective_rightbut_y;

    // liuli


    string birdview_file_name = run_dir + "birdview_size.txt";

    FILE* f_lut = fopen(birdview_file_name.c_str(),"r");
    if(f_lut == NULL)
    {
        cout<<birdview_file_name<<" not exist!"<<endl;
        return -1;
    }
    fscanf(f_lut,"front_pixel_height:%d   ",&front_height);
    fscanf(f_lut,"front_pixel_width:%d\n",&front_width);

    fscanf(f_lut,"rear_pixel_height:%d   ",&rear_height);
    fscanf(f_lut,"rear_pixel_width:%d\n",&rear_width);

    fscanf(f_lut,"left_pixel_height:%d   ",&left_height);
    fscanf(f_lut,"left_pixel_width:%d\n",&left_width);

    fscanf(f_lut,"right_pixel_height:%d   ",&right_height);
    fscanf(f_lut,"right_pixel_width:%d\n",&right_width);

    stitch_width = front_width;
    stitch_height = left_height;
    fclose(f_lut);

//liuli
    //把表都读出来
    string lut_file_name_front = run_dir + "birdview_front.bin";
    fp_lut_front = fopen(lut_file_name_front.c_str(),"rb");
    if(fp_lut_front == NULL)
    {
        cout<<lut_file_name_front<<" can not open!"<<endl;
        return -1;
    }

    string fusion_file_name_front = run_dir + "fusion_front.bin";
    fp_fusion_front = fopen(fusion_file_name_front.c_str(),"rb");
    if(fp_fusion_front == NULL)
    {
        cout<<fusion_file_name_front<<" can not open!"<<endl;
        return -1;
    }

    string lut_file_name_rear = run_dir + "birdview_rear.bin";
    fp_lut_rear = fopen(lut_file_name_rear.c_str(),"rb");
    if(fp_lut_rear == NULL)
    {
        cout<<lut_file_name_rear<<" can not open!"<<endl;
        return -1;
    }

    string fusion_file_name_rear = run_dir + "fusion_rear.bin";
    fp_fusion_rear = fopen(fusion_file_name_rear.c_str(),"rb");
    if(fp_fusion_rear == NULL)
    {
        cout<<fusion_file_name_rear<<" can not open!"<<endl;
        return -1;
    }

    string lut_file_name_left = run_dir + "birdview_left.bin";
    fp_lut_left = fopen(lut_file_name_left.c_str(),"rb");
    if(fp_lut_left == NULL)
    {
        cout<<lut_file_name_left<<" can not open!"<<endl;
        return -1;
    }

    string fusion_file_name_left = run_dir + "fusion_left.bin";
    fp_fusion_left = fopen(fusion_file_name_left.c_str(),"rb");
    if(fp_fusion_left == NULL)
    {
        cout<<fusion_file_name_left<<" can not open!"<<endl;
        return -1;
    }

    string lut_file_name_right = run_dir + "birdview_right.bin";
    fp_lut_right = fopen(lut_file_name_right.c_str(),"rb");
    if(fp_lut_right == NULL)
    {
        cout<<lut_file_name_right<<" can not open!"<<endl;
        return -1;
    }

    string fusion_file_name_right = run_dir + "fusion_right.bin";
    fp_fusion_right = fopen(fusion_file_name_right.c_str(),"rb");
    if(fp_fusion_right == NULL)
    {
        cout<<fusion_file_name_right<<" can not open!"<<endl;
        return -1;
    }
#if 1
    file_data_front = (uint32_t*)malloc(front_height*front_width*(sizeof(uint32_t)*2 + 4*sizeof(float)));
    fread(file_data_front,front_height*front_width*(sizeof(uint32_t)*2 + 4*sizeof(float)),1,fp_lut_front);
    fclose(fp_lut_front);

    file_data_rear = (uint32_t*)malloc(rear_height*rear_width*(sizeof(uint32_t)*2 + 4*sizeof(float)));
    fread(file_data_rear,rear_height*rear_width*(sizeof(uint32_t)*2 + 4*sizeof(float)),1,fp_lut_rear);
    fclose(fp_lut_rear);

    file_data_left = (uint32_t*)malloc(left_height*left_width*(sizeof(uint32_t)*2 + 4*sizeof(float)));
    fread(file_data_left,left_height*left_width*(sizeof(uint32_t)*2 + 4*sizeof(float)),1,fp_lut_left);
    fclose(fp_lut_left);

    file_data_right = (uint32_t*)malloc(right_height*right_width*(sizeof(uint32_t)*2 + 4*sizeof(float)));
    fread(file_data_right,right_height*right_width*(sizeof(uint32_t)*2 + 4*sizeof(float)),1,fp_lut_right);
    fclose(fp_lut_right);


    file_fusion_front = (float*)malloc(front_height * front_width * sizeof(float));
    fread(file_fusion_front, front_height * front_width * sizeof(float), 1, fp_fusion_front);
    fclose(fp_fusion_front);

    file_fusion_rear = (float*)malloc(rear_height * rear_width * sizeof(float));
    fread(file_fusion_rear, rear_height * rear_width * sizeof(float), 1, fp_fusion_rear);
    fclose(fp_fusion_rear);

    file_fusion_left = (float*)malloc(left_height * left_width * sizeof(float));
    fread(file_fusion_left, left_height * left_width * sizeof(float), 1, fp_fusion_left);
    fclose(fp_fusion_left);

    file_fusion_right = (float*)malloc(right_height * right_width * sizeof(float));
    fread(file_fusion_right, right_height * right_width * sizeof(float), 1, fp_fusion_right);
    fclose(fp_fusion_right);

#endif
    return 0;
}

int freePointer()
{
    FREE_POINTER(file_data_front);
    FREE_POINTER(file_data_rear);
    FREE_POINTER(file_data_left);
    FREE_POINTER(file_data_right);

    FREE_POINTER(file_fusion_front);
    FREE_POINTER(file_fusion_rear);
    FREE_POINTER(file_fusion_left);
    FREE_POINTER(file_fusion_right);

    return 0;
}

double get_polycoeff(double k[4])
{
    k[1] = poly_k1;
    k[2] = poly_k2;
    k[3] = poly_k3;
    k[4] = poly_k4;
}

typedef enum projectiveview_point_enum
{
    projectiveview_point_min = 0,
    projectiveview_point_lefttop_e,
    projectiveview_point_leftbut_e,
    projectiveview_point_righttop_e,
    projectiveview_point_rightbut_e,
    projectiveview_point_max,
}PROJECTIVE_POINT_ENUM;


static int bilinear_interpolation(IN int image_size[2], IN float pixel[2] , INOUT float weight[4]);
static int load_cam_intrinstic(cam_model_s *cam_model, IN VIEW_E view_num);
static int load_cam_extrinstic(INOUT double trans[3][4], IN VIEW_E view_num);
static int imagepixel_bilinear_generate(IN Mat raw_image, INOUT Mat undistort_image,int src_position[2], int dst_position[2],IN float weight[4]);
static int imagepixel_bilinear_fusion(IN Mat raw_image_1, IN Mat raw_image_2, INOUT Mat undistort_image, int src_position_1[2], int dst_position[2], IN float weight_1[4], int src_position_2[2], IN float weight_2[4], IN float fusion[2]);
static int lutWeight_biliner_generate(IN FILE* fp_lut, IN int position[2], IN float weight[4]);
static int world2pixel_lut(IN double world_position[3], IN int dst_position[2], INOUT double trans[3][4], IN Mat raw_image, INOUT Mat dst_image, IN FILE* fp_lut, IN VIEW_E birdview, IN double fusion_coeff);
static int world2pixel(IN double world_position[3], IN double trans[3][4], INOUT float pixel_position[2], IN VIEW_E view_index);
static int worldpoint2undistortpixel(IN VIEW_E view_index, IN PROJECTIVE_POINT_ENUM projective_point, IN float projective_fov[2],IN float projective_size[2], OUT float projective_pixel[2] );
static int image_stitchFusion(IN Mat stitch_front_image, IN Mat stitch_rear_image, IN Mat stitch_left_image, IN Mat stitch_right_image, IN bool front_use, IN bool rear_use, IN bool left_use, IN bool right_use, INOUT Mat stitch_fusion_image);
/*
 * author: cheng
 * date:
 * name: bilinear_interpolation
 * function:产生双线性插值系数
 * parametes:
 *          IN
 *          OUT
 *        INOUT
 *
 *return;
 * */
static int bilinear_interpolation(IN int image_size[2], IN float pixel[2] , INOUT float weight[4])
{
    int rtn = 0;
    float weightUpLeft;
    float weightUpRight;
    float weightDownLeft;
    float weightDownRight;

    float y = pixel[0];
    float x = pixel[1];

    int x0 = (int)x;
    int x1 = (int)x + 1;
    int y0 = (int)y;
    int y1 = (int)y + 1;

    int image_width = image_size[0];
    int image_height = image_size[1];

    x0 = x0 < 0 ? 0 : x0;
    x1 = x1 < 0 ? 0 : x1;
    y0 = y0 < 0 ? 0 : y0;
    y1 = y1 < 0 ? 0 : y1;

//    x0 = x0 > image_width - 1 ? image_width - 1 : x0;
//    x1 = x1 > image_width - 1 ? image_width - 1 : x1;
//    y0 = y0 > image_height - 1 ? image_height - 1 : y0;
//    y1 = y1 > image_height - 1 ? image_height - 1 : y1;


    weightUpLeft = (x1 - x) * (y1 - y);
    weightUpRight = (x - x0) * (y1 - y);
    weightDownLeft = (x1 - x) * (y - y0);
    weightDownRight = 1 - weightUpLeft - weightUpRight - weightDownLeft;

    weight[0] = weightUpLeft;
    weight[1] = weightUpRight;
    weight[2] = weightDownLeft;
    weight[3] = weightDownRight;

    return rtn;
}

/*
 * author: cheng
 * date:
 * name:imagepixel_bilinear_generate
 * function: 根据坐标点及双线性插值系数生成目标点像素值
 * parametes:
 *          IN
 *          OUT
 *        INOUT
 *
 *return;
 * */
static int imagepixel_bilinear_generate(IN Mat raw_image, INOUT Mat undistort_image, int src_position[2], int dst_position[2], IN float weight[4])
{
    int rtn = 0;
    int row = dst_position[0];
    int col = dst_position[1];

    int src_row = src_position[0];
    int src_col = src_position[1];

    int dst_channel = undistort_image.channels();

    float a1 = weight[0];
    float a2 = weight[1];
    float a3 = weight[2];
    float a4 = weight[3];

/*
    a1 = a1 < 0 ? 0 : a1;
    a2 = a2 < 0 ? 0 : a2;
    a3 = a3 < 0 ? 0 : a3;
    a4 = a4 < 0 ? 0 : a4;

    a1 = a1 > 1 ? 1 : a1;
    a2 = a2 > 1 ? 1 : a2;
    a3 = a3 > 1 ? 1 : a3;
    a4 = a4 > 1 ? 1 : a4;
*/
    if (dst_channel == 3) {

        Vec3b* p;
        Vec3b* p1;
        Vec3b* p2;
        p = undistort_image.ptr<Vec3b>(row);
        p1 = raw_image.ptr<Vec3b>(src_row);
        p2 = raw_image.ptr<Vec3b>(src_row + 1);

        p[col][0] =
                p1[src_col][0] * a1 + p1[src_col + 1][0] * a2 + p2[src_col][0] * a3 + p2[src_col + 1][0] * a4;

        p[col][1] =
                p1[src_col][1] * a1 + p1[src_col + 1][1] * a2 + p2[src_col][1] * a3 + p2[src_col + 1][1] * a4;

        p[col][2] =
                p1[src_col][2] * a1 + p1[src_col + 1][2] * a2 + p2[src_col][2] * a3 + p2[src_col + 1][2] * a4;


        p[col][0] = p[col][0] < 1 ? 0 : p[col][0];
        p[col][1] = p[col][1] < 1 ? 0 : p[col][1];
        p[col][2] = p[col][2] < 1 ? 0 : p[col][2];

        p[col][0] = p[col][0] > 254 ? 255 : p[col][0];
        p[col][1] = p[col][1] > 254 ? 255 : p[col][1];
        p[col][2] = p[col][2] > 254 ? 255 : p[col][2];

    }
    else
    {
        uchar* p;
        uchar* p1;
        uchar* p2;
        p = undistort_image.ptr<uchar>(row);
        p1 = raw_image.ptr<uchar>(src_row);
        p2 = raw_image.ptr<uchar>(src_row + 1);
        p[col] = p1[src_col] * a1 + p1[src_col + 1]* a2 + p2[src_col] * a3 + p2[src_col + 1] * a4;

    }
    return rtn;
}

static int imagepixel_bilinear_fusion(IN Mat raw_image_1, IN Mat raw_image_2, INOUT Mat undistort_image, int src_position_1[2], int dst_position[2], IN float weight_1[4], int src_position_2[2], IN float weight_2[4], IN float fusion[2])
{
    int rtn = 0;
    int row = dst_position[0];
    int col = dst_position[1];

    int src_row_1 = src_position_1[0];
    int src_col_1 = src_position_1[1];

    int src_row_2 = src_position_2[0];
    int src_col_2 = src_position_2[1];

    int dst_channel = undistort_image.channels();

    float a1_1 = weight_1[0];
    float a2_1 = weight_1[1];
    float a3_1 = weight_1[2];
    float a4_1 = weight_1[3];

    float a1_2 = weight_2[0];
    float a2_2 = weight_2[1];
    float a3_2 = weight_2[2];
    float a4_2 = weight_2[3];

    float fusion_1 = fusion[0];
    float fusion_2 = fusion[1];


    /*
    a1 = a1 < 0 ? 0 : a1;
    a2 = a2 < 0 ? 0 : a2;
    a3 = a3 < 0 ? 0 : a3;
    a4 = a4 < 0 ? 0 : a4;

    a1 = a1 > 1 ? 1 : a1;
    a2 = a2 > 1 ? 1 : a2;
    a3 = a3 > 1 ? 1 : a3;
    a4 = a4 > 1 ? 1 : a4;
*/
    if (dst_channel == 3) {
        Vec3b* p;

    Vec3b* p1;
    Vec3b* p1_1;
    Vec3b* p2;
    Vec3b* p2_1;
    p = undistort_image.ptr<Vec3b>(row);
    p1 = raw_image_1.ptr<Vec3b>(src_row_1);
    p1_1 = raw_image_1.ptr<Vec3b>(src_row_1 + 1);
    p2 = raw_image_2.ptr<Vec3b>(src_row_2);
    p2_1 = raw_image_2.ptr<Vec3b>(src_row_2 + 1);



        p[col][0] =
                (p1[src_col_1][0] * a1_1 + p1[src_col_1+1][0] * a2_1 +
                 p1_1[src_col_1][0] * a3_1 + p1_1[src_col_1+1][0] * a4_1)*fusion_1 +
                (p2[src_col_2][0] * a1_2 + p2[src_col_2+1][0] * a2_2 +
                 p2_1[src_col_2][0] * a3_2 + p2_1[src_col_2+1][0] * a4_2)*fusion_2;
        p[col][1] =
                (p1[src_col_1][1] * a1_1 + p1[src_col_1+1][1] * a2_1 +
                 p1_1[src_col_1][1] * a3_1 + p1_1[src_col_1+1][1] * a4_1)*fusion_1 +
                (p2[src_col_2][1] * a1_2 + p2[src_col_2+1][1] * a2_2 +
                 p2_1[src_col_2][1] * a3_2 + p2_1[src_col_2+1][1] * a4_2)*fusion_2;
        p[col][2] =
                (p1[src_col_1][2] * a1_1 + raw_image_1.ptr<Vec3b>(src_row_1)[src_col_1+1][2] * a2_1 +
                 p1_1[src_col_1][2] * a3_1 + p1_1[src_col_1+1][2] * a4_1)*fusion_1 +
                (p2[src_col_2][2] * a1_2 + p2[src_col_2+1][2] * a2_2 +
                 p2_1[src_col_2][2] * a3_2 + p2_1[src_col_2+1][2] * a4_2)*fusion_2;


        p[col][0] = p[col][0] < 1 ? 0 : p[col][0];
        p[col][1] = p[col][1] < 1 ? 0 : p[col][1];
        p[col][2] = p[col][2] < 1 ? 0 : p[col][2];

        p[col][0] = p[col][0] > 254 ? 255 : p[col][0];
        p[col][1] = p[col][1] > 254 ? 255 : p[col][1];
        p[col][2] = p[col][2] > 254 ? 255 : p[col][2];

    }
    else
    {
        uchar* p;

        uchar* p1;
        uchar* p1_1;
        uchar* p2;
        uchar* p2_1;
        p = undistort_image.ptr<uchar>(row);
        p1 = raw_image_1.ptr<uchar>(src_row_1);
        p1_1 = raw_image_1.ptr<uchar>(src_row_1 + 1);
        p2 = raw_image_2.ptr<uchar>(src_row_2);
        p2_1 = raw_image_2.ptr<uchar>(src_row_2 + 1);

        p[col] = p1[src_col_1] * a1_1 + p1[src_col_1 +1 ] * a2_1 +
                 p1_1[src_col_1] * a3_1 + p1_1[src_col_1 + 1] * a4_1 +
                 p2[src_col_2] * a2_1 + p2[src_col_2 +1 ] * a2_2 +
                 p2_1[src_col_2] * a3_2 + p2_1[src_col_2 + 1] * a4_2;
    }
    return rtn;
}

/*
 * author: cheng
 * date:
 * name:lutWeight_biliner_generate
 * function: 坐标点及线性插值系数保存为lut table
 * parametes:
 *          IN
 *          OUT
 *        INOUT
 *
 *return;
 * */
static int lutWeight_biliner_generate(IN FILE* fp_lut, IN int position[2], IN float weight[4])
{
    int rtn = 0;
    uint32_t row = position[0];
    uint32_t col = position[1];

    float a1 = weight[0];
    float a2 = weight[1];
    float a3 = weight[2];
    float a4 = weight[3];



    fwrite(&row,sizeof(uint32_t), 1, fp_lut);
    fwrite(&col,sizeof(uint32_t), 1, fp_lut);
    fwrite(&a1,sizeof(float), 1, fp_lut);
    fwrite(&a2,sizeof(float), 1, fp_lut);
    fwrite(&a3,sizeof(float), 1, fp_lut);
    fwrite(&a4,sizeof(float), 1, fp_lut);

    return rtn;

}

/*
 * author: cheng
 * date:
 * name:load_cam_instrinstic
 * function: 加载相机内参
 * parametes:
 *          IN
 *          OUT
 *        INOUT
 *
 *return;
 * */
static int load_cam_intrinstic(cam_model_s *cam_model, IN VIEW_E birdview)
{
    int rtn = 0;

    Mat K;
    Mat D;

    if (birdview == view_front_e)
    {
        fs["front_K"]>>K;
        fs["front_D"]>>D;

        cam_model->fx = K.at<double>(0,0);
        cam_model->cx = K.at<double>(0,2);
        cam_model->fy = K.at<double>(1,1);
        cam_model->cy = K.at<double>(1,2);

        cam_model->k1 = D.at<double>(0,0);
        cam_model->k2 = D.at<double>(1,0);
        cam_model->k3 = D.at<double>(2,0);
        cam_model->k4 = D.at<double>(3,0);
    }
    else if (birdview == view_rear_e)
    {
        fs["back_K"]>>K;
        fs["back_D"]>>D;

        cam_model->fx = K.at<double>(0,0);
        cam_model->cx = K.at<double>(0,2);
        cam_model->fy = K.at<double>(1,1);
        cam_model->cy = K.at<double>(1,2);

        cam_model->k1 = D.at<double>(0,0);
        cam_model->k2 = D.at<double>(1,0);
        cam_model->k3 = D.at<double>(2,0);
        cam_model->k4 = D.at<double>(3,0);
    }
    else if (birdview == view_left_e)
    {
        fs["left_K"]>>K;
        fs["left_D"]>>D;

        cam_model->fx = K.at<double>(0,0);
        cam_model->cx = K.at<double>(0,2);
        cam_model->fy = K.at<double>(1,1);
        cam_model->cy = K.at<double>(1,2);

        cam_model->k1 = D.at<double>(0,0);
        cam_model->k2 = D.at<double>(1,0);
        cam_model->k3 = D.at<double>(2,0);
        cam_model->k4 = D.at<double>(3,0);
    }
    else if (birdview == view_right_e)
    {
        fs["right_K"]>>K;
        fs["right_D"]>>D;

        cam_model->fx = K.at<double>(0,0);
        cam_model->cx = K.at<double>(0,2);
        cam_model->fy = K.at<double>(1,1);
        cam_model->cy = K.at<double>(1,2);

        cam_model->k1 = D.at<double>(0,0);
        cam_model->k2 = D.at<double>(1,0);
        cam_model->k3 = D.at<double>(2,0);
        cam_model->k4 = D.at<double>(3,0);
    }

    return rtn;
}

/*
 * author: cheng
 * date:
 * name:load_cam_extrinstic
 * function: 加载相机外参
 * parametes:
 *          IN
 *          OUT
 *        INOUT
 *
 *return;
 * */
int load_cam_extrinstic(INOUT double trans[3][4], IN VIEW_E view_num)
{
    int rtn = 0;

    Mat trans_m;

    if (view_num == view_front_e)
    {
        fs["front_trans"] >> trans_m;
    }
    else if (view_num == view_rear_e)
    {
        fs["back_trans"] >> trans_m;
    }
    else if (view_num == view_left_e)
    {
        fs["left_trans"] >> trans_m;
    }
    else if (view_num == view_right_e)
    {
        fs["right_trans"] >> trans_m;
    }
    if(xml_version)
    {
        trans[1][0] = trans_m.at<double>(0,0);
        trans[1][1] = trans_m.at<double>(0,1);
        trans[1][2] = trans_m.at<double>(0,2);
        trans[1][3] = trans_m.at<double>(0,3);//后轴中心，外参

        trans[0][0] = trans_m.at<double>(1,0);
        trans[0][1] = trans_m.at<double>(1,1);
        trans[0][2] = trans_m.at<double>(1,2);
        trans[0][3] = trans_m.at<double>(1,3);
    } else{
		trans[0][0] = trans_m.at<double>(0,0);
        trans[0][1] = trans_m.at<double>(0,1);
        trans[0][2] = trans_m.at<double>(0,2);
        trans[0][3] = trans_m.at<double>(0,3);//后轴中心，外参

        trans[1][0] = trans_m.at<double>(1,0);
        trans[1][1] = trans_m.at<double>(1,1);
        trans[1][2] = trans_m.at<double>(1,2);
        trans[1][3] = trans_m.at<double>(1,3);

    }

    trans[2][0] = trans_m.at<double>(2,0);
    trans[2][1] = trans_m.at<double>(2,1);
    trans[2][2] = trans_m.at<double>(2,2);
    trans[2][3] = trans_m.at<double>(2,3);
#if 0
    static int test = 0;

    if (test ==0) {
        test++;
        Mat R(3, 3, CV_32F);

        R.at<float>(0, 0) = trans[0][0];
        R.at<float>(0, 1) = trans[0][1];
        R.at<float>(0, 2) = trans[0][2];

        R.at<float>(1, 0) = trans[1][0];
        R.at<float>(1, 1) = trans[1][1];
        R.at<float>(1, 2) = trans[1][2];

        R.at<float>(2, 0) = trans[2][0];
        R.at<float>(2, 1) = trans[2][1];
        R.at<float>(2, 2) = trans[2][2];

        Mat RR(3, 3, CV_32F);

        Mat T(3, 1, CV_32F);

        T.at<float>(0, 0) = trans[0][3];
        T.at<float>(1, 0) = trans[1][3];
        T.at<float>(2, 0) = trans[2][3];

        transpose(R, RR);

        std::cout << R << endl;

        std::cout << T << endl;

        std::cout << RR << endl;

        Mat result = -RR * T;
        cout << "resutl:" << result << endl;
    }
#endif


    return rtn;
}

/*
 * author: cheng
 * date:
 * name:undistort_plane_image
 * function:生成去畸变平面透视图，并生成lut table
 * parametes:
 *          IN: Mat raw_image 原始鱼眼图
 *          IN: float fov[2] 透视展开时，fov视场角大小
 *          IN: IN VIEW_E view_index 前、后、左、右四个相机枚举
 *          OUT:
 *        INOUT：Mat undistort_image 透视展开后，目标图像
 *
 *return;
 * */
int undistort_plane_image(IN Mat raw_image, INOUT Mat undistort_image, IN float fov[2],IN VIEW_E view_index)
{
    int rtn = 0;
    int src_width = raw_image.cols;
    int src_height= raw_image.rows;
    int dst_width = undistort_image.cols;
    int dst_height= undistort_image.rows;

    int dst_channel = undistort_image.channels();
    float fov_h = fov[0];
    float fov_v = fov[1];

    int image_size[2];

    image_size[0] = src_width;
    image_size[1] = src_height;

    cam_model_s cam_model;
    load_cam_intrinstic(&cam_model, view_index);

    float weight[4];

    float cam_x = 0;
    float cam_y = 0;
    float cam_z = 0;
    float cam_world_point[3];
    float pixel[2];

    float cam_z_init = 100; //视场深度

    float tan_h =tan(fov_h * PI/180/2); //单位视场深度下对应的水平距离，先把FOV劈半，然后转化成弧度，然后tan运算
    float tan_v =tan(fov_v * PI/180/2); //同理为垂直方向上的。

    float  pixel_hight_dis = (cam_z_init * tan_v) / (dst_height / 2); //垂直方向上，平均每个像素占的物理距离。视场深度乘以单位视场深度对应的水平距离。然后除以像素个数，注意还是劈半。
    float  pixel_width_dis = (cam_z_init * tan_h) / (dst_width / 2);  //水平方向上，平均每个像素占的物理距离。同理为垂直方向上的。

    int src_position[2];
    int dst_position[2];

    FILE* fp_lut;

    if(view_index == view_front_e)
    {
        string file_name = run_dir + "projective_front.bin";
        fp_lut = fopen(file_name.c_str(),"wb+");
        matrix_rowoffset =  front_matrix_rowoffset;
        matrix_coloffset =  front_matrix_coloffset;

     //   fp_lut = fopen("./lut/projective_front.bin","wb+");
    }
    else if (view_index == view_rear_e)
    {
        string file_name = run_dir + "projective_rear.bin";
        fp_lut = fopen(file_name.c_str(),"wb+");
        matrix_rowoffset =  rear_matrix_rowoffset;
        matrix_coloffset =  rear_matrix_coloffset;

     //   fp_lut = fopen("./lut/projective_rear.bin","wb+");
    }
    else if (view_index == view_left_e)
    {
        string file_name = run_dir + "projective_left.bin";
        fp_lut = fopen(file_name.c_str(),"wb+");
        matrix_rowoffset =  left_matrix_rowoffset;
        matrix_coloffset =  left_matrix_coloffset;

      //  fp_lut = fopen("./lut/projective_left.bin","wb+");
    }
    else if (view_index == view_right_e)
    {
        string file_name = run_dir + "projective_right.bin";
        fp_lut = fopen(file_name.c_str(),"wb+");
        matrix_rowoffset =  right_matrix_rowoffset;
        matrix_coloffset =  right_matrix_coloffset;

     //   fp_lut = fopen("./lut/projective_right.bin","wb+");
    }

    int dst_image_size[2] = {dst_width, dst_height};
    float matirx[3][3] = {0};

    undistort_generate_matirx(matrix_rowoffset, matrix_coloffset, fov, dst_image_size, matirx, view_index);  //0.12ms

    float fxi = matirx[0][0];
    float cxi = matirx[0][2];
    float fyi = matirx[1][1];
    float cyi = matirx[1][2];

    for(int row = 0; row < dst_height; row++)
    {
        dst_position[0] = row;

//        int row_offset = row - 170;

        for(int col = 0; col < dst_width; col++)
        {
            dst_position[1] = col;

//            int col_offset = col -50;
//            cam_x = (col - dst_width / 2) * pixel_width_dis;
//            cam_y = (row - dst_height / 2) * pixel_hight_dis;
//            cam_z = cam_z_init;

            cam_x = (col * fxi + cxi) * cam_z_init;
//            cam_x = (col  * fxi -  50* fxi + cxi) * cam_z_init;
//            cxi = cxi - matrix_coloffset*fxi;

            cam_y = (row * fyi + cyi) * cam_z_init;
//            cam_y = (row* fyi - 170* fyi  + cyi) * cam_z_init;
//            cyi = cyi - matrix_rowoffset*fyi;
            cam_z =  cam_z_init;

            cam_world_point[1] = cam_x;
            cam_world_point[0] = cam_y;
            cam_world_point[2] = cam_z;  //至此得到相机坐标系下的空间物理点坐标。

            //鱼眼相机投影模型，空间点投影到鱼眼图上的像素坐标。
            cam2pixel(cam_model, cam_world_point, pixel);

            //由于投影过程可能会出现小数点像素坐标位置，所以用双线性插值，取得亚像素前后左右四个像素块的权重。
            bilinear_interpolation(image_size, pixel, weight);

//            src_position[0] = int(pixel[1]);
//            src_position[1] = int(pixel[0]);

            //赋值一下，把亚像素坐标转化成int类型像素坐标。至此，双层循环中目标图像中各个待填充的像素，已经找到原始鱼眼图上所对应的像素位置和权重。
            src_position[0] = int(pixel[0]);
            src_position[1] = int(pixel[1]);

            //这里填充目标图像的一个像素。参数很规整，原始图，目标图，原始图像素位置，目标图像素位置，权重
            imagepixel_bilinear_generate(raw_image, undistort_image, src_position, dst_position, weight);

            //将本次目标图上的像素坐标位置循环所对应的原始鱼眼图上的像素坐标位置和权重保存为LUT表。
            lutWeight_biliner_generate(fp_lut, src_position, weight);

        }
    }

    fclose(fp_lut);

    fp_lut = NULL;

    return rtn;
}


#if __test_intri__
/*
 * author: liu
 * date:
 * name: undistort_cylinder_image
 * function:生成柱面展开透视图，并生成lut table
 * parametes:
 *          IN: Mat raw_image 原始鱼眼图
 *          IN: float fov[2] 透视展开时，fov视场角大小
 *          IN: IN VIEW_E view_index 前、后、左、右四个相机枚举
 *        INOUT: Mat column_image 柱面透视展开后，目标图像
 *return;
 * */
int undistort_cylinder_image(IN Mat raw_image, INOUT Mat column_image, IN float fov[2],IN VIEW_E view_index)
{
    int rtn = 0;
    int src_width = raw_image.cols;
    int src_height= raw_image.rows;

    float fov_h = fov[0];
    float fov_v = fov[1];

    float radius = 1000;

    int dst_width = column_image.cols;
    int dst_height= column_image.rows;

    int image_size[2];

    image_size[0] = src_width;
    image_size[1] = src_height;

    cam_model_s cam_model;
    load_cam_intrinstic(&cam_model, view_index);

    float weight[4];

    float cam_x = 0;
    float cam_y = 0;
    float cam_z = 0;
    float cam_world_point[3];
    float pixel[2];

    int src_position[2];
    int dst_position[2];

    //相机坐标系下的物理点
    float d_angle_h =  fov_h / dst_width;    //角度制   每一目标像素对应的角度
    float d_angle_v =  fov_v / dst_height;    //角度制

    FILE* fp_lut;

    if(view_index == view_front_e)
    {
        string file_name = run_dir + "cylinder_front.bin";
        fp_lut = fopen(file_name.c_str(),"wb+");

        //   fp_lut = fopen("./lut/projective_front.bin","wb+");
    }
    else if (view_index == view_rear_e)
    {
        string file_name = run_dir + "cylinder_rear.bin";
        fp_lut = fopen(file_name.c_str(),"wb+");
        //   fp_lut = fopen("./lut/projective_rear.bin","wb+");
    }
    else if (view_index == view_left_e)
    {
        string file_name = run_dir + "cylinder_left.bin";
        fp_lut = fopen(file_name.c_str(),"wb+");
        //  fp_lut = fopen("./lut/projective_left.bin","wb+");
    }
    else if (view_index == view_right_e)
    {
        string file_name = run_dir + "cylinder_right.bin";
        fp_lut = fopen(file_name.c_str(),"wb+");
        //   fp_lut = fopen("./lut/projective_right.bin","wb+");
    }


    for(int row = 0; row < dst_height; row++)
    {
        dst_position[0] = row;
//        cam_y = -radius * tan((fov_v/2 - row * d_angle_v) * PI/180); //world_y
        cam_x = -radius * sin((fov_v/2 - row * d_angle_v) * PI/180);

        for(int col = 0; col < dst_width; col++)
        {
            dst_position[1] = col;
//            cam_x = radius * sin((fov_h/2 - col * d_angle_h) * PI/180);
//            cam_z =  radius * cos((fov_h/2 - col * d_angle_h) * PI/180);;

            cam_y = -radius * tan((fov_h/2 - col * d_angle_h) * PI/180); //world_y
            cam_z =  radius * cos((fov_v/2 - row * d_angle_v) * PI/180) * cos((fov_h/2 - col * d_angle_h) * PI/180);;


            cam_world_point[0] = cam_x;
            cam_world_point[1] = cam_y;
            cam_world_point[2] = cam_z;  //至此得到相机坐标系下的空间物理点坐标。


            //鱼眼相机投影模型，空间点投影到鱼眼图上的像素坐标。
            cam2pixel(cam_model, cam_world_point, pixel);

            //由于投影过程可能会出现小数点像素坐标位置，所以用双线性插值，取得亚像素前后左右四个像素块的权重。
            bilinear_interpolation(image_size, pixel, weight);

            //赋值一下，把亚像素坐标转化成int类型像素坐标。至此，双层循环中目标图像中各个待填充的像素，已经找到原始鱼眼图上所对应的像素位置和权重。
            src_position[0] = int(pixel[0]);
            src_position[1] = int(pixel[1]);

            //这里填充目标图像的一个像素。参数很规整，原始图，目标图，原始图像素位置，目标图像素位置，权重
            imagepixel_bilinear_generate(raw_image, column_image, src_position, dst_position, weight);

            //将本次目标图上的像素坐标位置循环所对应的原始鱼眼图上的像素坐标位置和权重保存为LUT表。
            lutWeight_biliner_generate(fp_lut, src_position, weight);

        }
    }

    fclose(fp_lut);

    fp_lut = NULL;

    return rtn;
}

#else

/*
 * author: 外参柱面展开
 * date:
 * name:undistort_cylinder_image
 * function:生成去畸变透视图，并生成lut table
 * parametes:
 *          IN: Mat raw_image 原始鱼眼图
 *          IN: float fov[2] 透视展开时，fov视场角大小
 *          IN: IN VIEW_E view_index 前、后、左、右四个相机枚举
 *        INOUT：Mat column_image 柱面透视展开后，目标图像
 *return;
 * */
int undistort_cylinder_image(IN Mat raw_image, INOUT Mat column_image, IN float fov[2],IN VIEW_E view_index)
{
    int rtn = 0;
    int src_width = raw_image.cols;
    int src_height= raw_image.rows;

    float fov_h = fov[0] * PI/180;
    float fov_v = fov[1] * PI/180;

    int dst_width = column_image.cols;
    int dst_height= column_image.rows;

    int dst_position[2] = {0};

    int image_size[2];

    image_size[0] = src_width;
    image_size[1] = src_height;

    cam_model_s cam_model;
    load_cam_intrinstic(&cam_model, view_index);

    double trans[3][4];
    load_cam_extrinstic(trans, view_index);

    float weight[4];

    double world_coordinate[3];
    int src_position[2];
    float pixel[2];

    float d_angle_h =  fov_h / dst_width;    //角度制   每一目标像素对应的角度
    float d_angle_v =  fov_v / dst_height;    //角度制

    FILE* fp_lut;


    // 赋值。R直接赋值
    Mat srcR(3,3,CV_32F);

    srcR.at<float>(0,0) = float(trans[0][0]);
    srcR.at<float>(0,1) = float(trans[0][1]);
    srcR.at<float>(0,2) = float(trans[0][2]);

    srcR.at<float>(1,0) = float(trans[1][0]);
    srcR.at<float>(1,1) = float(trans[1][1]);
    srcR.at<float>(1,2) = float(trans[1][2]);


    srcR.at<float>(2,0) = float(trans[2][0]);
    srcR.at<float>(2,1) = float(trans[2][1]);
    srcR.at<float>(2,2) = float(trans[2][2]);

    //T由行向量变为列向量
    Mat srcT(3,1,CV_32F);

    srcT.at<float>(0,0) = float(trans[0][3]);
    srcT.at<float>(1,0) = float(trans[1][3]);
    srcT.at<float>(2,0) = float(trans[2][3]);

    Mat verification_result = -(srcR.inv())*srcT;
    cout << "verification_result:" << verification_result<< endl;

    float pose_t[3] = {verification_result.at<float>(0,0), verification_result.at<float>(1,0),
                       verification_result.at<float>(2,0)};

    float radius = pose_t[2]/tan(fov_v/2); // unit is mm

    if(view_index == view_front_e)
    {
        string file_name = run_dir + "cylinder_front.bin";
        fp_lut = fopen(file_name.c_str(),"wb+");
        //   fp_lut = fopen("./lut/projective_front.bin","wb+");
    }
    else if (view_index == view_rear_e)
    {
        string file_name = run_dir + "cylinder_rear.bin";
        fp_lut = fopen(file_name.c_str(),"wb+");
        //   fp_lut = fopen("./lut/projective_rear.bin","wb+");
    }
    else if (view_index == view_left_e)
    {
        string file_name = run_dir + "cylinder_left.bin";
        fp_lut = fopen(file_name.c_str(),"wb+");
        //  fp_lut = fopen("./lut/projective_left.bin","wb+");
    }
    else if (view_index == view_right_e)
    {
        string file_name = run_dir + "cylinder_right.bin";
        fp_lut = fopen(file_name.c_str(),"wb+");
        //   fp_lut = fopen("./lut/projective_right.bin","wb+");
    }


    for(int row = 0; row < dst_height; row++)     // row
    {
        dst_position[0] = row;
        if(view_index == view_front_e)
        {
            world_coordinate[2] = pose_t[2] - radius * tan(fov_v/2 - row * d_angle_v);     //z
        }
        else if(view_index == view_right_e)
        {
            world_coordinate[2] = pose_t[2] + radius * tan(fov_v/2 - row * d_angle_v);     //z
        }
        for(int col = 0; col < dst_width; col++)   //col
        {

            dst_position[1] = col;

            if(view_index == view_front_e)
            {
                world_coordinate[0] = pose_t[0] + radius * sin(fov_h/2 - col * d_angle_h);     //x
                world_coordinate[1] = (pose_t[1] - radius * cos(fov_h/2 - col * d_angle_h));     //y
            }
            else if(view_index == view_right_e)
            {
                world_coordinate[0] = (pose_t[0] + radius * cos(fov_h/2 - col * d_angle_h));     //y
                world_coordinate[1] = (pose_t[1] + radius * sin(fov_h/2 - col * d_angle_h));     //x
            }

            //物理坐标转位原图像素坐标
            world2pixel(world_coordinate, trans, pixel, view_index);

            //由于投影过程可能会出现小数点像素坐标位置，所以用双线性插值，取得亚像素前后左右四个像素块的权重。
            bilinear_interpolation(image_size, pixel, weight);

            //赋值一下，把亚像素坐标转化成int类型像素坐标。至此，双层循环中目标图像中各个待填充的像素，已经找到原始鱼眼图上所对应的像素位置和权重。
            src_position[0] = int(pixel[0]);
            src_position[1] = int(pixel[1]);

            //这里填充目标图像的一个像素。参数很规整，原始图，目标图，原始图像素位置，目标图像素位置，权重
            imagepixel_bilinear_generate(raw_image, column_image, src_position, dst_position, weight);

            //将本次目标图上的像素坐标位置循环所对应的原始鱼眼图上的像素坐标位置和权重保存为LUT表。
            lutWeight_biliner_generate(fp_lut, src_position, weight);

        }
    }

    fclose(fp_lut);

    fp_lut = NULL;

    return rtn;
}

#endif




/*
 * author: cheng
 * date:
 * name: undistort_generate_matirx
 * function:产生从像素到相机坐标变换矩阵
 * parametes:
 *          IN: float fov[2]
 *          IN: int dst_image_size[2]
 *          IN: int src_image_size[2]
 *          OUT:
 *        INOUT：float matirx[3][3]
 *
 *return;
 * */
void undistort_generate_matirx(IN int matrix_rowoffset, IN int matrix_coloffset, IN float fov[2], IN int dst_image_size[2], INOUT float matrix[3][3], IN VIEW_E view_index)
{
    float fov_h = fov[0];
    float fov_v = fov[1];
    float tan_h =tan(fov_h * PI/180/2); //单位视场深度下对应的水平距离，先把FOV劈半，然后转化成弧度，然后tan运算
    float tan_v =tan(fov_v * PI/180/2); //同理为垂直方向上的。

    int dst_width = dst_image_size[0];
    int dst_height = dst_image_size[1];

    float fxi = tan_h / (dst_width/2);
    float fyi = tan_v / (dst_height/2);

    float cxi = (-dst_width/2)  * (tan_h / (dst_width/2));
    float cyi = (-dst_height/2) * (tan_v / (dst_height/2));

    cxi = cxi - matrix_coloffset*fxi;
    cyi = cyi - matrix_rowoffset*fyi;

    float fx = 1/fxi;
    float fy = 1/fyi;
    float cx = -cxi/fxi;
    float cy = -cyi/fyi;


    matrix[0][0] = fxi;
    matrix[0][2] = cxi;
    matrix[1][1] = fyi;
    matrix[1][2] = cyi;
    matrix[2][2] = 0;

    FILE* f_lut;
    if(view_index == view_front_e)
    {
        string file_name = run_dir + "transfor_matrix_front.txt";
        f_lut = fopen(file_name.c_str(),"w+");
    }
    else if (view_index == view_rear_e)
    {
        string file_name = run_dir + "transfor_matrix_rear.txt";
        f_lut = fopen(file_name.c_str(),"w+");
    }
    else if (view_index == view_left_e)
    {
        string file_name = run_dir + "transfor_matrix_left.txt";
        f_lut = fopen(file_name.c_str(),"w+");
    }
    else if (view_index == view_right_e)
    {
        string file_name = run_dir + "transfor_matrix_right.txt";
        f_lut = fopen(file_name.c_str(),"w+");
    }
    fprintf(f_lut,"fx:%-12f",fx);
    fprintf(f_lut,"fy:%-12f",fy);
    fprintf(f_lut,"cx:%-12f",cx);
    fprintf(f_lut,"cy:%-12f\n",cy);

    fprintf(f_lut,"fxi:%-12f",fxi);
    fprintf(f_lut,"fyi:%-12f",fyi);
    fprintf(f_lut,"cxi:%-12f",cxi);
    fprintf(f_lut,"cyi:%-12f\n",cyi);

    fprintf(f_lut,"fov_h:%-12f",fov_h);
    fprintf(f_lut,"fov_v:%-12f\n",fov_v);

    fprintf(f_lut,"dst_width:%-12d",dst_width);
    fprintf(f_lut,"dst_height:%-12d\n",dst_height);

    fprintf(f_lut,"matrix_rowoffset:%-12d",  matrix_rowoffset);
    fprintf(f_lut,"matrix_coloffset:%-12d\n", matrix_coloffset);

    fclose(f_lut);

    return ;
}


/*
 * author: cheng
 * date:
 * name:multiview_undistort_plane_image
 * function:生成多视角去畸变平面透视图，并生成lut table
 * parametes:
 *          IN: Mat raw_image 原始鱼眼图
 *          IN: float fov[2] 透视展开时，fov视场角大小
 *          IN: VIEW_E view_index_e 前、后、左、右四个相机枚举
 *          OUT:
 *        INOUT：Mat undistort_image 透视展开后，目标图像
 *
 *return;
 * */
int multiview_undistort_plane_image(IN Mat raw_image, INOUT Mat undistort_image, IN float fov[2], IN VIEW_E view_index_e)
{
    int rtn = 0;
    int src_width = raw_image.cols;
    int src_height= raw_image.rows;
    int dst_width = undistort_image.cols;
    int dst_height= undistort_image.rows;

    int row = 0;
    int col = 0;

    int dst_channel = undistort_image.channels();
    float fov_h = fov[0];
    float fov_v = fov[1];

    int image_size[2];

    image_size[0] = src_width;
    image_size[1] = src_height;

    cam_model_s cam_model;
    load_cam_intrinstic(&cam_model, view_index_e);

    float weight[4];

    float cam_x = 0;
    float cam_y = 0;
    float cam_z = 0;
    float cam_world_point[3];
    float pixel[2];

    float cam_z_init = 100;

    float tan_h = tan(fov_h * PI/180/2);
    float cos_h = cos(fov_h * PI/180/2);
    float sin_h = sin(fov_h * PI/180/2);
    float tan_v = tan(fov_v * PI/180/2);
    float cos_v = cos(fov_v * PI/180/2);

    float fov_view_top = 30; //上侧，fov大小
    float fov_view_bot = 30;//下侧，fov大小
    float fov_view_left = 30; //左侧，fov大小
    float fov_view_right = 30;//右侧，fov大小

    float z_angle = 45;
    float tan_z =tan(z_angle * PI/180);
    float tan_fov_vec = tan(fov_view_top * PI/180/2);
    float sin_fov_vec = sin(fov_view_top * PI/180/2);

    int src_position[2];
    int dst_position[2];

    uint16_t file_reserve_data = 0;
    float z_offset;
    int vec_view_height = 85;
    int hor_view_width = 85;

    float cam_z_vec_init = cam_z_init / cos_v; //折叠视角下，垂直fov最边缘边的长度,亦为原fov最外边边的长度
    float cam_z_hor_init = cam_z_init / cos_h; //折叠视角下，水平fov最边缘边的长度,亦为原fov最外边边的长度

    //中间部分
    float vec_view_height_mid = dst_height - 2 * vec_view_height; //中间平面透视展开的像素高度
    float hor_view_width_mid = dst_width - 2 * hor_view_width;
    float pixel_vec_hight_dis_mid = (cam_z_init * tan_v) / (vec_view_height_mid / 2);
    float pixel_hor_width_dis_mid =  (cam_z_hor_init * sin_h) / (hor_view_width_mid / 2); //中间部分，水平方向上平均每个像素占的物理距离
    //float pixel_width_dis = (cam_z_init * tan_h) / (dst_width / 2);

    float cam_y_view_top_mid = (vec_view_height - vec_view_height_mid / 2) * pixel_vec_hight_dis_mid; //正常视角下,即中间部分，y方向上边沿的坐标
    float cam_y_view_bot_mid = ((dst_height - vec_view_height-1) - vec_view_height_mid / 2) * pixel_vec_hight_dis_mid; //正常视角下,即中间部分，y方向下边沿的坐标
    float cam_x_view_left_mid = (hor_view_width - hor_view_width_mid / 2) * pixel_hor_width_dis_mid;; //正常视角下,即中间部分，x方向左边沿的坐标
    float cam_x_view_right_mid = ((dst_width - hor_view_width-1) - hor_view_width_mid / 2) * pixel_hor_width_dis_mid;//正常视角下，即中间部分，x方向右边沿的坐标

    //上、下部分
    float pixel_vec_hight_dis_top =  (cam_z_vec_init * sin_fov_vec) / (vec_view_height / 2);//上侧部分，垂直方向平均每个像素占的物理距离
    float pixel_hor_width_dis_top =  pixel_hor_width_dis_mid;//上侧部分，水平方向平均每个像素占的物理距离
    float pixel_vec_hight_dis_bottom = pixel_vec_hight_dis_top;  //下侧部分，垂直方向平均每个像素占的物理距离
    float pixel_hor_width_dis_bottom =  pixel_hor_width_dis_mid;//下侧部分，水平方向平均每个像素占的物理距离

    //左、右部分
    float pixel_hor_width_dis_left =  (cam_z_hor_init * sin_h) / (hor_view_width / 2);//左侧部分，水平方向平均每个像素占的物理距离
    float pixel_vec_height_dis_left = pixel_vec_hight_dis_mid;//左侧部分，水平方向平均每个像素占的物理距离
    float pixel_hor_width_dis_right =  pixel_hor_width_dis_left;//右侧部分，水平方向平均每个像素占的物理距离
    float pixel_vec_height_dis_right = pixel_vec_hight_dis_mid;//右侧部分，水平方向平均每个像素占的物理距离

    FILE* fp_lut;

    if(view_index_e == view_front_e)
    {
        string file_name = run_dir + "multiview_front.bin";
        fp_lut = fopen(file_name.c_str(),"wb+");

     //   fp_lut = fopen("./lut/multiview_front.bin","wb+");
    }
    else if (view_index_e == view_rear_e)
    {
        string file_name = run_dir + "multiview_front.bin";
        fp_lut = fopen(file_name.c_str(),"wb+");

    //    fp_lut = fopen("./lut/multiview_rear.bin","wb+");
    }
    else if (view_index_e == view_left_e)
    {
        string file_name = run_dir + "multiview_left.bin";
        fp_lut = fopen(file_name.c_str(),"wb+");

    //    fp_lut = fopen("./lut/multiview_left.bin","wb+");
    }
    else if (view_index_e == view_right_e)
    {
        string file_name = run_dir + "multiview_right.bin";
        fp_lut = fopen(file_name.c_str(),"wb+");

     //   fp_lut = fopen("./lut/multiview_right.bin","wb+");
    }

    //上面旋转相机角度展开
    for(row = 0; row < vec_view_height; row++)
    {
        dst_position[0] = row;

        for(int data_index = 0; data_index < hor_view_width*2*2; data_index++)
        {
            fwrite(&file_reserve_data,sizeof(uint16_t), 1, fp_lut);
        }

        for(col = hor_view_width; col < dst_width - hor_view_width; col++)
        {
            dst_position[1] = col;

            cam_x = (col - hor_view_width_mid / 2) * pixel_hor_width_dis_top;
            cam_y = (row - vec_view_height) * pixel_vec_hight_dis_top * cos(fov_view_top * PI/180) + cam_y_view_top_mid;
            z_offset = tan_z * (vec_view_height-row);
            cam_z = cam_z_init - z_offset;

            cam_world_point[1] = cam_x;
            cam_world_point[0] = cam_y;
            cam_world_point[2] = cam_z;

            cam2pixel(cam_model, cam_world_point, pixel);

            bilinear_interpolation(image_size, pixel,weight);

            src_position[0] = int(pixel[0]);
            src_position[1] = int(pixel[1]);

            imagepixel_bilinear_generate(raw_image, undistort_image, src_position, dst_position, weight);

            lutWeight_biliner_generate(fp_lut, src_position, weight);

        }

     //   fwrite(&file_reserve_data,sizeof(uint16_t), hor_view_width*2*2, fp_lut);
    }

    //中间正常平面透视展开部分
    for(row = vec_view_height; row < (dst_height - vec_view_height); row++)
    {
        dst_position[0] = row;

        //左侧折叠
        for(col = 0; col < hor_view_width; col++)
        {
            dst_position[1] = col;

            // cam_x = (col - dst_width / 2) * pixel_width_dis;
            cam_x = (col - hor_view_width) * pixel_hor_width_dis_left * cos(fov_view_left * PI/180) + cam_x_view_left_mid;
            cam_y = (row - vec_view_height_mid / 2) * pixel_vec_height_dis_left;
            cam_z = cam_z_init;

            z_offset = tan_z * (hor_view_width-col);
            cam_z = cam_z_init - z_offset;

            cam_world_point[1] = cam_x;
            cam_world_point[0] = cam_y;
            cam_world_point[2] = cam_z;

            cam2pixel(cam_model, cam_world_point, pixel);

            bilinear_interpolation(image_size, pixel,weight);

            src_position[0] = int(pixel[0]);
            src_position[1] = int(pixel[1]);

            imagepixel_bilinear_generate(raw_image, undistort_image, src_position, dst_position,weight);

            lutWeight_biliner_generate(fp_lut, src_position,weight);
        }

        //中间正常平面展开部分
        for(col = hor_view_width; col < dst_width - hor_view_width; col++)
        {
            dst_position[1] = col;

           // cam_x = (col - dst_width / 2) * pixel_width_dis;
            cam_x = (col - hor_view_width_mid / 2) * pixel_hor_width_dis_mid;
            cam_y = (row - vec_view_height_mid / 2) * pixel_vec_hight_dis_mid;
            cam_z = cam_z_init;

            cam_world_point[1] = cam_x;
            cam_world_point[0] = cam_y;
            cam_world_point[2] = cam_z;

            cam2pixel(cam_model, cam_world_point, pixel);

            bilinear_interpolation(image_size, pixel,weight);

            src_position[0] = int(pixel[0]);
            src_position[1] = int(pixel[1]);

            imagepixel_bilinear_generate(raw_image, undistort_image, src_position, dst_position,weight);

            lutWeight_biliner_generate(fp_lut, src_position,weight);

        }

        //右侧折叠
        for(col =(dst_width - hor_view_width); col < dst_width; col++)
        {
            dst_position[1] = col;

            // cam_x = (col - dst_width / 2) * pixel_width_dis;
            cam_x = (col - (dst_width - hor_view_width)) * pixel_hor_width_dis_right * cos(fov_view_right * PI/180) + cam_x_view_right_mid;
            cam_y = (row - vec_view_height_mid / 2) * pixel_vec_height_dis_right;
            cam_z = cam_z_init;

            cam_world_point[1] = cam_x;
            cam_world_point[0] = cam_y;
            cam_world_point[2] = cam_z;

            cam2pixel(cam_model, cam_world_point, pixel);

            bilinear_interpolation(image_size, pixel,weight);

            src_position[0] = int(pixel[0]);
            src_position[1] = int(pixel[1]);

            imagepixel_bilinear_generate(raw_image, undistort_image, src_position, dst_position,weight);

            lutWeight_biliner_generate(fp_lut, src_position,weight);
        }

    }

    //下面旋转相机角度展开
    for(row = (dst_height - vec_view_height); row < dst_height; row++)
    {
        dst_position[0] = row;

       // for(col = 0; col < dst_width; col++)
        for(col = hor_view_width; col < dst_width - hor_view_width; col++)
        {
            dst_position[1] = col;

            //cam_x = (col - dst_width / 2) * pixel_width_dis;
            cam_x = (col - hor_view_width_mid / 2) * pixel_hor_width_dis_bottom;
            cam_y = (row - (dst_height - vec_view_height) ) * pixel_vec_hight_dis_bottom * cos(fov_view_bot * PI/180) + cam_y_view_bot_mid;

            z_offset = tan_z * (row - (dst_height - vec_view_height));
            cam_z = cam_z_init - z_offset;

            cam_world_point[1] = cam_x;
            cam_world_point[0] = cam_y;
            cam_world_point[2] = cam_z;

            cam2pixel(cam_model, cam_world_point, pixel);

            bilinear_interpolation(image_size, pixel,weight);

            src_position[0] = int(pixel[0]);
            src_position[1] = int(pixel[1]);

            imagepixel_bilinear_generate(raw_image, undistort_image, src_position, dst_position,weight);

            lutWeight_biliner_generate(fp_lut, src_position,weight);
        }
    }

    fclose(fp_lut);

    /*

    for(int row = 0; row < dst_height; row++)
    {
        dst_position[0] = row;

        for(int col = 0; col < dst_width; col++)
        {
            dst_position[1] = col;

            cam_x = (col - dst_width / 2) * pixel_width_dis;
          //  cam_y = (row - dst_height / 2) * pixel_hight_dis;
            cam_y = (row - planeview_height / 2) * pixel_hight_dis;

            if (row < vec_view_height)
            {
              //  cam_y = cam_y * cos(fov_view * PI/180);

                cam_y = (row - vec_view_height) * pixel_vec_hight_dis;
                cam_y = cam_y * cos(fov_view * PI/180) + cam_y_view_init;

                z_offset = tan_z * (vec_view_height-row);
                cam_z = cam_z_init - z_offset;
            }
            else if(row > (dst_height - vec_view_height))
            {
              //  cam_y = cam_y * cos(fov_view * PI/180);

                cam_y = (row - (dst_height - vec_view_height)) * pixel_vec_hight_dis;
                cam_y = cam_y * cos(fov_view * PI/180) - cam_y_view_init;

                z_offset = tan_z * (row - (dst_height - vec_view_height));
                cam_z = cam_z_init - z_offset;
            }
            else
            {
                cam_x = (col - dst_width / 2) * pixel_width_dis;
                cam_y = (row - vec_view_height - planeview_height / 2) * pixel_hight_dis;
                cam_z = cam_z_init;
            }

            cam_world_point[1] = cam_x;
            cam_world_point[0] = cam_y;
            cam_world_point[2] = cam_z;

            cam2pixel(cam_model, cam_world_point, pixel);

            bilinear_interpolation(image_size, pixel,weight);

            src_position[0] = int(pixel[0]);
            src_position[1] = int(pixel[1]);

            imagepixel_bilinear_generate(raw_image, undistort_image, src_position, dst_position,weight);

            lutWeight_biliner_generate(fp_lut, src_position,weight);

        }
    }

    */

    return rtn;
}

/*
 * author: cheng
 * date:
 * name:world2pixel
 * function:生成俯视图，并生成lut table
 * parametes:
 *          IN: double world_position[3] 物理坐标系坐标
 *          IN: double trans[3][4] 转换矩阵
 *          OUT:
 *        INOUT：float pixel_position[2] 像素坐标
 *return:
 * */
int world2pixel(IN double world_position[3], IN double trans[3][4], INOUT float pixel_position[2], IN VIEW_E view_index)
{
    int rtn;
    cam_model_s cam_model;
    load_cam_intrinstic(&cam_model, view_index);
    float cam_world_point[3] = {0};

    cam_world_point[1] = trans[0][0] * world_position[0] + trans[0][1] * world_position[1] + trans[0][2] * world_position[2] + trans[0][3];
    cam_world_point[0] = trans[1][0] * world_position[0] + trans[1][1] * world_position[1] + trans[1][2] * world_position[2] + trans[1][3];
    cam_world_point[2] = trans[2][0] * world_position[0] + trans[2][1] * world_position[1] + trans[2][2] * world_position[2] + trans[2][3];

    cam2pixel(cam_model, cam_world_point, pixel_position);

    return rtn;

}

/*
 * author: cheng
 * date:
 * name:birdview_image_generate
 * function:生成俯视图，并生成lut table
 * parametes:
 *          IN: Mat raw_image 原始鱼眼图
 *          IN:  VIEW_E view_index视图相机枚举,前、后、左、右相机
 *          IN bool stitch_select_b 是否需要生成融合系数
 *          IN float stretch_coef 车位倾斜程度拉伸系数
 *          IN float yCoef 车位纵向拉宽系数
 *return: Mat birdview_image 俯视图像
 * */
Mat birdview_image_generate(IN Mat raw_image, IN VIEW_E view_index, IN bool stitch_select_b, IN float stretch_coef, IN float yCoef)
{
    double trans[3][4] = {0};
    int src_width = raw_image.cols;
    int src_height= raw_image.rows;
    int image_size[2] = {0};
    image_size[0] = src_width;
    image_size[1] = src_height;

    int world_height = front_world_view + rear_world_view + car_world_height;   // 物理尺度mm
    int world_width = left_world_view + right_world_view + car_world_width;

    //每个像素在xy方向上所表征的物理距离。
    double dy = world_height / double(left_pixel_height);
    double dx = dy;

    //计算前后左右展开后的俯视图的分辨率。根据设定的左视图高度等比例计算。
    int left_pixel_width = round(left_world_view * left_pixel_height/world_height);// key：保证长宽比

    int front_pixel_height = round(front_world_view * left_pixel_height/world_height);
    int front_pixel_width = round(world_width * left_pixel_height/world_height);

    int rear_pixel_height = round(rear_world_view * left_pixel_height/world_height);
    int rear_pixel_width = front_pixel_width;

    int right_pixel_height = left_pixel_height;
    int right_pixel_width = round(right_world_view * left_pixel_height/world_height);

    string file_name = run_dir + "birdview_size.txt";
    FILE* f_lut = fopen(file_name.c_str(),"w+");   //以纯文本形式读写或新建文件，清零

    //将分辨率写入文件。
    fprintf(f_lut,"front_pixel_height:%-6d",front_pixel_height);  //不足6位，右侧补齐空格；超过6位，按实际输出
    fprintf(f_lut,"front_pixel_width:%-6d\n",front_pixel_width);

    fprintf(f_lut,"rear_pixel_height:%-6d",rear_pixel_height);
    fprintf(f_lut,"rear_pixel_width:%-6d\n",rear_pixel_width);

    fprintf(f_lut,"left_pixel_height:%-6d",left_pixel_height);
    fprintf(f_lut,"left_pixel_width:%-6d\n",left_pixel_width);

    fprintf(f_lut,"right_pixel_height:%-6d",right_pixel_height);
    fprintf(f_lut,"right_pixel_width:%-6d\n",left_pixel_width);
    fclose(f_lut);

    Mat birdview_image;
    double world_position[3] = {0};
    float pixel_position[2] = {0};
    int src_position[2] = {0};
    int dst_position[2] = {0};
    float weight[4] = {0};
    FILE* fp_lut;
    FILE* fusion_lut;

    double fusion_angel_left = 40;
    double fusion_angel_right = 50;
    float fusion_coeff = 1.0; //融合系数
    double default_fusion_coeff = 1.0;

    //front
    if (view_front_e == view_index)
    {
        //创建空白俯视结果图，往里面填充像素值。
        birdview_image = Mat::zeros(front_pixel_height,front_pixel_width, CV_8UC3);

        if(stitch_select_b)  //判断是否生成融合区域的融合系数
        {
            string file_name = run_dir + "fusion_front.bin";
            fusion_lut = fopen(file_name.c_str(),"wb+");    //以二进制形式读写或新建文件，清零
        }

        string file_name = run_dir + "birdview_front.bin";
        fp_lut = fopen(file_name.c_str(),"wb+");

        for (int row = 0; row < front_pixel_height; row++) {
            dst_position[0] = row;

            for (int col = 0; col < front_pixel_width; col++) {
                dst_position[1] = col;

                world_position[0] = (col - front_pixel_width / 2) * dx;
                world_position[1] = (front_pixel_height - row) * dy + car_axle_coord;
                //拉伸
//                world_position[1] = stretch_coef * world_position[0] + yCoef * world_position[1];
                world_position[1] += stretch_coef * world_position[0];
                world_position[1] = yCoef * world_position[1];
                world_position[2] = 0;

                //生成融合系数
                if(stitch_select_b)
                {
                    //右、前融合区,前融合系数
                    if(col >= (front_pixel_width - right_pixel_width) && col < front_pixel_width)
                    {
                        if(atan2((front_pixel_height-row), (col-(front_pixel_width - right_pixel_width))) > fusion_angel_right*M_PI/180)
                        {
                            fusion_coeff = 1.0;
                        }
                        else if(atan2((front_pixel_height-row), (col-(front_pixel_width - right_pixel_width))) < fusion_angel_left*M_PI/180)
                        {
                            fusion_coeff = 0.0;
                        }
                        else if(atan2((front_pixel_height-row), (col-(front_pixel_width - right_pixel_width))) > fusion_angel_left*M_PI/180 && atan2((front_pixel_height-row), (col-(front_pixel_width - right_pixel_width))) < fusion_angel_right*M_PI/180)
                        {
                            double front_coefficient = (atan2((front_pixel_height-row), (col-(front_pixel_width - right_pixel_width)))-fusion_angel_left*M_PI/180)/(fusion_angel_right*M_PI/180 - fusion_angel_left*M_PI/180);
                            double right_coefficient = 1*front_coefficient;
                            fusion_coeff = front_coefficient;
                        }
                    }
                    //左、前融合区中,前融合系数
                   else if ( atan2((front_pixel_height - row), (left_pixel_width - col)) < fusion_angel_left*M_PI/180 )
                    {
                        fusion_coeff = 0;
                    }
                    else if(atan2((front_pixel_height - row), (left_pixel_width - col)) > fusion_angel_right*M_PI/180)
                    {
                        fusion_coeff = 1.0;
                    }
                    else if(atan2((front_pixel_height - row), (left_pixel_width - col)) >= fusion_angel_left*M_PI/180 && atan2((front_pixel_height-row), (left_pixel_width - col)) <=fusion_angel_right*M_PI/180 )
                    {
                        double front_coefficient = (atan2((front_pixel_height - row), (left_pixel_width - col))-fusion_angel_left*M_PI/180)/(fusion_angel_right*M_PI/180 - fusion_angel_left*M_PI/180);
                        double left_coefficient = 1*front_coefficient;

                        fusion_coeff = front_coefficient;
                    }

                    fwrite(&fusion_coeff,sizeof(float), 1, fusion_lut);

                //    default_fusion_coeff = fusion_coeff;
                }
                world2pixel_lut(world_position, dst_position, trans, raw_image, birdview_image, fp_lut, view_index, default_fusion_coeff);
            }
        }
    }
    //rear
    else if (view_rear_e == view_index)
    {
        birdview_image = Mat::zeros(rear_pixel_height,rear_pixel_width, CV_8UC3);

        if(stitch_select_b)
        {
            string file_name = run_dir + "fusion_rear.bin";
            fusion_lut = fopen(file_name.c_str(),"wb+");
        }

        string file_name = run_dir + "birdview_rear.bin";
        fp_lut = fopen(file_name.c_str(),"wb+");

        for (int row = 0; row < rear_pixel_height; row++) {
            dst_position[0] = row;

            for (int col = 0; col < rear_pixel_width; col++) {
                dst_position[1] = col;

                world_position[0] = (col - rear_pixel_width / 2) * dx;
                world_position[1] = -row * dy + car_axle_coord - car_world_height;

                //拉伸
//                world_position[1] = stretch_coef * world_position[0] + yCoef * world_position[1];
                world_position[1] += stretch_coef * world_position[0];
                world_position[1] = yCoef * world_position[1];
                world_position[2] = 0;

                //生成融合系数
                if(stitch_select_b)
                {
                    //右、后融合区，后融合系数
                    if(col > (rear_pixel_width - right_pixel_width))
                    {
                        int rear_col =col - (rear_pixel_width - right_pixel_width);

                        //if(atan2(row , rear_col) < fusion_angel_left*M_PI/180)
                        if(atan2(rear_col, row) < fusion_angel_left*M_PI/180)
                        {
                            fusion_coeff = 1.0;
                        }else if(atan2(rear_col, row) > fusion_angel_right*M_PI/180)
                        {
                            fusion_coeff = 0.0;
                        }else if(atan2(rear_col, row) >= fusion_angel_left*M_PI/180 && atan2(rear_col, row) <= fusion_angel_right*M_PI/180)
                        {
                            double right_coefficient = (fusion_angel_right*M_PI/180-atan2(rear_col, row))/(fusion_angel_right*M_PI/180 - fusion_angel_left*M_PI/180);
                            double back_coefficient = 1*right_coefficient;

                            fusion_coeff = back_coefficient;
                        }
                    }
                    //左、后融合区,后融合系数
                    else if (atan2(row , (left_pixel_width - col)) < fusion_angel_left * M_PI / 180)
                    {
                        fusion_coeff = 0.0;
                    }
                    else if (atan2(row, (left_pixel_width - col)) > fusion_angel_right * M_PI / 180)
                    {
                        fusion_coeff = 1.0;
                    }
                    else if (atan2(row, (left_pixel_width - col)) > fusion_angel_left * M_PI / 180 && atan2(row, (left_pixel_width - col)) < fusion_angel_right * M_PI / 180)
                    {
                        double back_coefficient =(atan2(row, (left_pixel_width - col)) - fusion_angel_left * M_PI / 180) / (fusion_angel_right*M_PI/180 - fusion_angel_left*M_PI/180);
                        double left_coefficient = 1 * back_coefficient;

                        fusion_coeff = back_coefficient;
                    }

                    fwrite(&fusion_coeff,sizeof(float), 1, fusion_lut);
                 //   default_fusion_coeff = fusion_coeff;
                }

                world2pixel_lut(world_position, dst_position, trans, raw_image, birdview_image, fp_lut, view_index, default_fusion_coeff);

            }
        }
    }
    //left
    else if (view_left_e == view_index)
    {
        double origin_y_pixel =(front_world_view + car_axle_coord)/dy;
        birdview_image = Mat::zeros(left_pixel_height,left_pixel_width, CV_8UC3);

        if(stitch_select_b)
        {
            string file_name = run_dir + "fusion_left.bin";
            fusion_lut = fopen(file_name.c_str(),"wb+");
        }

        string file_name = run_dir + "birdview_left.bin";
        fp_lut = fopen(file_name.c_str(),"wb+");

        for (int row = 0; row < left_pixel_height; row++) {
            dst_position[0] = row;

            for (int col = 0; col < left_pixel_width; col++) {
                dst_position[1] = col;

                world_position[0] = (col - left_pixel_width) * dx - car_world_width/2;
                world_position[1] = (origin_y_pixel - row) * dy;

                //拉伸
//                world_position[1] = stretch_coef * world_position[0] + yCoef * world_position[1];
                world_position[1] += stretch_coef * world_position[0];
                world_position[1] = yCoef * world_position[1];
                world_position[2] = 0;

                //生存融合系数
                if(stitch_select_b)
                {
                    //左、后融合区,左融合系数
                    if(row > (left_pixel_height-rear_pixel_height))
                    {
                        int left_row = row - (left_pixel_height-rear_pixel_height);

                        if (atan2(left_row , (left_pixel_width - col)) < fusion_angel_left * M_PI / 180)
                        {
                            fusion_coeff = 1.0;
                        }
                        else if (atan2(left_row, (left_pixel_width - col)) > fusion_angel_right * M_PI / 180)
                        {
                            fusion_coeff = 0.0;
                        }
                        else if (atan2(left_row, (left_pixel_width - col)) > fusion_angel_left * M_PI / 180 && atan2(left_row, (left_pixel_width - col)) < fusion_angel_right * M_PI / 180)
                        {
                            double back_coefficient =(atan2(left_row, (left_pixel_width - col)) - fusion_angel_left * M_PI / 180) / (fusion_angel_right*M_PI/180 - fusion_angel_left*M_PI/180);
                            double left_coefficient = 1- back_coefficient;

                            fusion_coeff = left_coefficient;
                        }
                    }
                    //左、前融合区中,左侧融合系数
                    else if ( atan2((front_pixel_height - row), (left_pixel_width - col)) < fusion_angel_left*M_PI/180 )
                    {
                        fusion_coeff = 1;
                    }
                    else if(atan2((front_pixel_height - row), (left_pixel_width - col)) > fusion_angel_right*M_PI/180)
                    {
                        fusion_coeff = 0;
                    }
                    else if(atan2((front_pixel_height - row), (left_pixel_width - col)) >= fusion_angel_left*M_PI/180 && atan2((front_pixel_height-row), (left_pixel_width - col)) <=fusion_angel_right*M_PI/180 )
                    {
                        double front_coefficient = (atan2((front_pixel_height - row), (left_pixel_width - col))-fusion_angel_left*M_PI/180)/(fusion_angel_right*M_PI/180 - fusion_angel_left*M_PI/180);
                        double left_coefficient = 1-front_coefficient;

                        fusion_coeff = left_coefficient;
                    }

                    fwrite(&fusion_coeff,sizeof(float), 1, fusion_lut);
                //    default_fusion_coeff = fusion_coeff;
                }

                world2pixel_lut(world_position, dst_position, trans, raw_image, birdview_image, fp_lut, view_index, default_fusion_coeff);

            }
        }
    }
    //right
    else if (view_right_e == view_index)
    {
        double origin_y_pixel =(front_world_view + car_axle_coord)/dy;
        birdview_image = Mat::zeros(right_pixel_height,right_pixel_width, CV_8UC3);

        if(stitch_select_b)
        {
            string file_name = run_dir + "fusion_right.bin";
        //    string file_name = run_dir + "stitchview_right.bin";
            fusion_lut = fopen(file_name.c_str(),"wb+");
        }

        string file_name = run_dir + "birdview_right.bin";
        fp_lut = fopen(file_name.c_str(),"wb+");

        for (int row = 0; row < right_pixel_height; row++) {
            dst_position[0] = row;

            for (int col = 0; col < right_pixel_width; col++) {
                dst_position[1] = col;
                world_position[0] = col * dx + car_world_width/2;
                world_position[1] = (origin_y_pixel - row) * dy;

                //拉伸
//                world_position[1] = stretch_coef * world_position[0] + yCoef * world_position[1];
                world_position[1] += stretch_coef * world_position[0];
                world_position[1] = yCoef * world_position[1];
                world_position[2] = 0;

                //生成融合系数
                if(stitch_select_b)
                {
                    //右、后融合区,右融合系数
                    if(row > (right_pixel_height - rear_pixel_height))
                    {
                        int right_row = row - (right_pixel_height - rear_pixel_height);

                        if(atan2(right_row , col) < fusion_angel_left*M_PI/180)
                        {
                            fusion_coeff = 1.0;
                        }else if(atan2(right_row, col) > fusion_angel_right*M_PI/180)
                        {
                            fusion_coeff = 0.0;
                        }else if(atan2(right_row, col) >= fusion_angel_left*M_PI/180 && atan2(right_row, col) <= fusion_angel_right*M_PI/180)
                        {
                            double right_coefficient = (fusion_angel_right*M_PI/180-atan2(right_row, col))/(fusion_angel_right*M_PI/180 - fusion_angel_left*M_PI/180);
                            double back_coefficient = 1*right_coefficient;

                            fusion_coeff = right_coefficient;
                        }
                    }
                    //右、前融合区,右融合系数
                    else if(atan2((front_pixel_height-row), col) > fusion_angel_right*M_PI/180)
                    {
                        fusion_coeff = 0.0;
                    }
                    else if(atan2((front_pixel_height-row), col) < fusion_angel_left*M_PI/180)
                    {
                        fusion_coeff = 1.0;
                    }
                    else if(atan2((front_pixel_height-row), col) > fusion_angel_left*M_PI/180 && atan2((front_pixel_height-row), col) < fusion_angel_right*M_PI/180)
                    {
                        double front_coefficient_7 = (atan2((front_pixel_height-row), col)-fusion_angel_left*M_PI/180)/(fusion_angel_right*M_PI/180 - fusion_angel_left*M_PI/180);
                        double right_coefficient_7 = 1-front_coefficient_7;

                        fusion_coeff = right_coefficient_7;
                    }

                    fwrite(&fusion_coeff,sizeof(float), 1, fusion_lut);
                //    default_fusion_coeff = fusion_coeff;
                }

                world2pixel_lut(world_position, dst_position, trans, raw_image, birdview_image, fp_lut, view_index, default_fusion_coeff);

            }
        }
        fclose(fp_lut);
        fclose(fusion_lut);
    }
    return birdview_image;
}


/*
 * author: cheng
 * date:
 * name:world2pixel_lut
 * function: 物理坐标生成像素坐标,并写lut表
 * parametes:
 *          IN：
 *          OUT
 *        INOUT：
 *
 *return;
 * */
static int world2pixel_lut(IN double world_position[3], IN int dst_position[2], INOUT double trans[3][4], IN Mat raw_image, INOUT Mat dst_image, IN FILE* fp_lut, IN VIEW_E birdview, IN double fusion_coeff)
{
    int rtn;
    load_cam_extrinstic(trans, birdview);
    float pixel_position[2] = {0};
    float weight[4] = {0};
    int src_position[2] = {0};

    int src_width = raw_image.cols;
    int src_height= raw_image.rows;
    int image_size[2] = {0};
    image_size[0] = src_width;
    image_size[1] = src_height;

    world2pixel(world_position, trans, pixel_position, birdview);

    bilinear_interpolation(image_size, pixel_position, weight);
    src_position[0] = int(pixel_position[0]);
    src_position[1] = int(pixel_position[1]);

    weight[0] = weight[0] * fusion_coeff;
    weight[1] = weight[1] * fusion_coeff;
    weight[2] = weight[2] * fusion_coeff;
    weight[3] = weight[3] * fusion_coeff;

    imagepixel_bilinear_generate(raw_image, dst_image, src_position, dst_position, weight);

    lutWeight_biliner_generate(fp_lut, src_position, weight);

    return rtn;
}


/*
 * author: cheng
 * date:
 * name:image_stitchFusion
 * function: 使用lut table，生成相应的环视融合拼接图
 * parametes:
 *          IN： Mat front_raw_image 前俯视图
 *          IN： Mat rear_raw_image 后俯视图
 *          IN： Mat left_raw_image 左俯视图
 *          IN： Mat right_raw_image 右俯视图
 *          IN: bool front_use 是否使用前数据
 *          IN: bool rear_use  是否使用后数据
 *          IN: bool left_use  是否使用左数据
 *          IN: bool right_use  是否使用右数据
 *          OUT
 *        INOUT： Mat stitch_fusion_image
 *
 *return: ;
 * */
static int image_stitchFusion(IN Mat stitch_front_image, IN Mat stitch_rear_image, IN Mat stitch_left_image, IN Mat stitch_right_image, IN bool front_use, IN bool rear_use, IN bool left_use, IN bool right_use, INOUT Mat stitch_fusion_image)
{
    int rtn = 0;
//    int stitch_height = stitch_fusion_image.rows;
//    int stitch_width = stitch_fusion_image.cols;

    //采用融合拼接
    if(front_use && rear_use && left_use && right_use)
    {
        for(int row = 0; row < stitch_height; row++)
        {
            for(int col = 0; col < stitch_width; col++)
            {
                if(row < front_height) //上部分    //扩大中间灰色区域
                {
                    //前、左融合
                    if(col < left_width)
                    {
                        stitch_fusion_image.ptr<Vec3b>(row)[col] = stitch_front_image.ptr<Vec3b>(row)[col] + stitch_left_image.ptr<Vec3b>(row)[col];
                    }
                    else if(col > (front_width - right_width)) //前、右融合
                    {
                        int right_col = col - (front_width - right_width);
                        stitch_fusion_image.ptr<Vec3b>(row)[col] = stitch_front_image.ptr<Vec3b>(row)[col] + stitch_right_image.ptr<Vec3b>(row)[right_col];
                    }
                    else//前、中间部分
                    {
                        stitch_fusion_image.ptr<Vec3b>(row)[col] = stitch_front_image.ptr<Vec3b>(row)[col];
                    }

                }
                else if(row > (left_height - rear_height))//下部分 去掉底部车尾白边
                {
                    int rear_row = row - (left_height - rear_height);
                    if(col < left_width)
                    {
                        stitch_fusion_image.ptr<Vec3b>(row)[col] = stitch_rear_image.ptr<Vec3b>(rear_row)[col] + stitch_left_image.ptr<Vec3b>(row)[col];
                    }
                    else if(col > (rear_width - right_width))
                    {
                        int right_col = col - (front_width - right_width);
                        stitch_fusion_image.ptr<Vec3b>(row)[col] = stitch_rear_image.ptr<Vec3b>(rear_row)[col] + stitch_right_image.ptr<Vec3b>(row)[right_col];
                    }
                    else
                    {
                        stitch_fusion_image.ptr<Vec3b>(row)[col] = stitch_rear_image.ptr<Vec3b>(rear_row)[col];
                    }
                }
                else //中间部分
                {
                    //左侧部分
                    if(col < (left_width))
                    {
                        stitch_fusion_image.ptr<Vec3b>(row)[col] = stitch_left_image.ptr<Vec3b>(row)[col];
                    }
                        //右侧部分
                    else if(col > (front_width - right_width))
                    {
                        int right_col = col - (front_width - right_width);
                        stitch_fusion_image.ptr<Vec3b>(row)[col] = stitch_right_image.ptr<Vec3b>(row)[right_col];
                    }
                }
            }
        }

    }
    else
    {
        for(int row = 0; row < stitch_height; row++)
        {
            for(int col = 0; col < stitch_width; col++)
            {
                if(row < front_height) //上部分
                {
                    //前、左融合
                    if(col < left_width)
                    {
                        uchar* data = stitch_front_image.ptr<uchar>(row);
                        uchar data0 = data[col];

                        if(front_use && left_use)
                        {
                            stitch_fusion_image.ptr<Vec3b>(row)[col] = stitch_front_image.ptr<Vec3b>(row)[col] + stitch_left_image.ptr<Vec3b>(row)[col];
                        }
                        else if (front_use)
                        {
                            stitch_fusion_image.ptr<Vec3b>(row)[col] = stitch_front_image.ptr<Vec3b>(row)[col];
                        }
                        else if(left_use)
                        {
                            stitch_fusion_image.ptr<Vec3b>(row)[col] = stitch_left_image.ptr<Vec3b>(row)[col];
                        }

                    }
                    else if(col > (front_width - right_width)) //前、右融合
                    {
                        int right_col = col - (front_width - right_width);

                        uchar* data = stitch_front_image.ptr<uchar>(row);
                        uchar data0 = data[col];

                        if(front_use && right_use)
                        {
                            stitch_fusion_image.ptr<Vec3b>(row)[col] = stitch_front_image.ptr<Vec3b>(row)[col] + stitch_right_image.ptr<Vec3b>(row)[right_col];
                        }
                        else if(front_use)
                        {
                            stitch_fusion_image.ptr<Vec3b>(row)[col] = stitch_front_image.ptr<Vec3b>(row)[col];
                        }
                        else if(right_use)
                        {
                            stitch_fusion_image.ptr<Vec3b>(row)[col] = stitch_right_image.ptr<Vec3b>(row)[right_col];
                        }

                    }
                    else//前、中间部分
                    {
                        stitch_fusion_image.ptr<Vec3b>(row)[col] = stitch_front_image.ptr<Vec3b>(row)[col];
                    }

                }
                else if(row > (left_height - rear_height))//下部分
                {
                    int rear_row = row - (left_height - rear_height);
                    if(col < left_width)  //后、左融合
                    {
                        uchar* data = stitch_rear_image.ptr<uchar>(rear_row);
                        uchar data0 = data[col];

                        if (rear_use && left_use)
                        {
                          stitch_fusion_image.ptr<Vec3b>(row)[col] = stitch_rear_image.ptr<Vec3b>(rear_row)[col] + stitch_left_image.ptr<Vec3b>(row)[col];
                        }
                        else if(rear_use)
                        {
                            stitch_fusion_image.ptr<Vec3b>(row)[col] = stitch_rear_image.ptr<Vec3b>(rear_row)[col];
                        }
                        else if(left_use)
                        {
                            stitch_fusion_image.ptr<Vec3b>(row)[col] = stitch_left_image.ptr<Vec3b>(row)[col];
                        }

                    }
                    else if(col > (rear_width - right_width)) //后、右融合
                    {
                        int right_col = col - (front_width - right_width);

                        uchar* data = stitch_rear_image.ptr<uchar>(rear_row);
                        uchar data0 = data[col];

                        if(rear_use && right_use)
                        {
                          stitch_fusion_image.ptr<Vec3b>(row)[col] = stitch_rear_image.ptr<Vec3b>(rear_row)[col ] + stitch_right_image.ptr<Vec3b>(row)[right_col];
                        }
                        else if(rear_use)
                        {
                            stitch_fusion_image.ptr<Vec3b>(row)[col] = stitch_rear_image.ptr<Vec3b>(rear_row)[col];
                        }
                        else if(right_use)
                        {
                            stitch_fusion_image.ptr<Vec3b>(row)[col] = stitch_right_image.ptr<Vec3b>(row)[right_col];
                        }

                    }
                    else
                    {
                        stitch_fusion_image.ptr<Vec3b>(row)[col] = stitch_rear_image.ptr<Vec3b>(rear_row)[col];
                    }
                }
                else //中间部分
                {
                    //左侧部分
                    if(col < (left_width))
                    {
                       if( left_use) {
                           stitch_fusion_image.ptr<Vec3b>(row)[col] = stitch_left_image.ptr<Vec3b>(row)[col];
                       }
                    }
                        //右侧部分
                    else if(col > (front_width - right_width))
                    {
                        if(right_use) {
                            int right_col = col - (front_width - right_width);
                            stitch_fusion_image.ptr<Vec3b>(row)[col] = stitch_right_image.ptr<Vec3b>(row)[right_col];

                        }
                    }
                }
            }
        }
    }

    return rtn;

}

/*
 * author: cheng
 * date:
 * name:lutTable_generate_stitchFusion
 * function: 使用lut table，生成相应的环视融合拼接图
 * parametes:
 *          IN： Mat front_raw_imagbev_Table[fusion_pic1][i][j].wt_uplefte 前原始鱼眼图
 *          IN： Mat rear_raw_image 后原始鱼眼图
 *          IN： Mat left_raw_image 左原始鱼眼图
 *          IN： Mat right_raw_image 右原始鱼眼图
 *          IN: bool front_use 是否使用前数据
 *          IN: bool rear_use  是否使用后数据
 *          IN: bool left_use  是否使用左数据
 *          IN: bool right_use  是否使用右数据
 *          OUT
 *        INOUT：
 *
 *return: 生成的俯视融合图;
 * */
Mat lutTable_generate_stitchFusion(IN Mat front_raw_image, IN Mat rear_raw_image, IN Mat left_raw_image, IN Mat right_raw_image,IN bool front_use, IN bool rear_use, IN bool left_use, IN bool right_use)
{
//    int stitch_width = front_width;
//    int stitch_height = left_height;

    Mat stitch_fusion_image(stitch_height, stitch_width, CV_8UC3, Scalar(B,G,R));

    for(int row = 0; row < stitch_height; row++)
    {
        for(int col = 0; col < stitch_width; col++)
        {
            if(row < front_height) //上部分
            {
                if(col < left_width)   //前、左融合 0
                {
                    int row_offset_front = row * front_width * 6;
                    int row_offset_left = row * left_width * 6;
                    int col_offset = col * 6;

                    int src_row_front = *(file_data_front + row_offset_front+ col_offset);// x
                    int src_col_front = *(file_data_front + row_offset_front + col_offset+ 1); //y
                    int src_row_left = *(file_data_left + row_offset_left+ col_offset);
                    int src_col_left = *(file_data_left + row_offset_left + col_offset+ 1);

                    int fusion_offset_front = row * front_width + col;
                    float fusion_front = *(file_fusion_front + fusion_offset_front);
                    int fusion_offset_left = row * left_width + col;
                    float fusion_left = *(file_fusion_left + fusion_offset_left);

                    if((!front_use)&&(!left_use))
                    {
                        fusion_front = 0;
                        fusion_left = 0;
                    }

                    if (DEBUG_bilinear) {
                        float weight_1[4];
                        weight_1[0] = *(float*)(file_data_front + row_offset_front + col_offset+ 2);
                        weight_1[1] = *(float*)(file_data_front + row_offset_front + col_offset+ 3);
                        weight_1[2] = *(float*)(file_data_front + row_offset_front + col_offset+ 4);
                        weight_1[3] = *(float*)(file_data_front + row_offset_front + col_offset+ 5);

                        int src_position_1[2];
                        src_position_1[0] = src_row_front;
                        src_position_1[1] = src_col_front;

                        float weight_2[4];
                        weight_2[0] = *(float*)(file_data_left + row_offset_left+ col_offset+ 2);
                        weight_2[1] = *(float*)(file_data_left + row_offset_left+ col_offset+ 3);
                        weight_2[2] = *(float*)(file_data_left + row_offset_left+ col_offset+ 4);
                        weight_2[3] = *(float*)(file_data_left + row_offset_left+ col_offset+ 5);

                        int src_position_2[2];
                        src_position_2[0] = src_row_left;
                        src_position_2[1] = src_col_left;

                        int dst_position[2];
                        dst_position[0] = row;
                        dst_position[1] = col;

                        float fusion[2] = {0};
                        fusion[0] = fusion_front;
                        fusion[1] = fusion_left;

                        imagepixel_bilinear_fusion(front_raw_image, left_raw_image, stitch_fusion_image,src_position_1, dst_position, weight_1, src_position_2, weight_2, fusion);
                    } else {
                        stitch_fusion_image.ptr<Vec3b>(row)[col] =
                                front_raw_image.ptr<Vec3b>(src_row_front)[src_col_front] * fusion_front +
                                left_raw_image.ptr<Vec3b>(src_row_left)[src_col_left] * fusion_left;
                    }
                }
                else if(col > (front_width - right_width)) //前、右融合 2
                {
                    int row_offset_front = row * front_width * 6;
                    int col_offset_front = col * 6;
                    int row_offset_right = row * right_width * 6;
                    int col_offset_right = (col - (front_width - right_width)) * 6;

                    int src_row_front = *(file_data_front + row_offset_front+ col_offset_front);
                    int src_col_front = *(file_data_front + row_offset_front + col_offset_front+ 1);

                    int src_row_right = *(file_data_right + row_offset_right+ col_offset_right);
                    int src_col_right = *(file_data_right + row_offset_right + col_offset_right+ 1);

                    int fusion_offset_front = row * front_width + col;
                    float fusion_front = *(file_fusion_front + fusion_offset_front);

                    int fusion_offset_right = row * right_width +  (col - (front_width - right_width));
                    float fusion_right = *(file_fusion_right + fusion_offset_right);

                    if((!front_use)&&(!right_use))
                    {
                        fusion_front = 0;
                        fusion_right = 0;
                    }

                    if (DEBUG_bilinear) {
                        float weight_1[4];
                        weight_1[0] = *(float*)(file_data_front + row_offset_front + col_offset_front+ 2);
                        weight_1[1] = *(float*)(file_data_front + row_offset_front + col_offset_front+ 3);
                        weight_1[2] = *(float*)(file_data_front + row_offset_front + col_offset_front+ 4);
                        weight_1[3] = *(float*)(file_data_front + row_offset_front + col_offset_front+ 5);

                        int src_position_1[2];
                        src_position_1[0] = src_row_front;
                        src_position_1[1] = src_col_front;

                        float weight_2[4];
                        weight_2[0] = *(float*)(file_data_right + row_offset_right+ col_offset_right+ 2);
                        weight_2[1] = *(float*)(file_data_right + row_offset_right+ col_offset_right+ 3);
                        weight_2[2] = *(float*)(file_data_right + row_offset_right+ col_offset_right+ 4);
                        weight_2[3] = *(float*)(file_data_right + row_offset_right+ col_offset_right+ 5);

                        int src_position_2[2];
                        src_position_2[0] = src_row_right;
                        src_position_2[1] = src_col_right;

                        int dst_position[2];
                        dst_position[0] = row;
                        dst_position[1] = col;

                        float fusion[2] = {0};
                        fusion[0] = fusion_front;
                        fusion[1] = fusion_right;


                        imagepixel_bilinear_fusion(front_raw_image, right_raw_image, stitch_fusion_image,src_position_1, dst_position, weight_1, src_position_2, weight_2, fusion);
                    }else{
                        stitch_fusion_image.ptr<Vec3b>(row)[col] = front_raw_image.ptr<Vec3b>(src_row_front)[src_col_front]*fusion_front + right_raw_image.ptr<Vec3b>(src_row_right)[src_col_right]*fusion_right;
                    }
                }
                else//前、中间部分  1
                {
                    if(front_use)
                    {
                        int row_offset_front = row * front_width * 6;
                        int col_offset_front = col * 6;

                        int src_row_front = *(file_data_front + row_offset_front+ col_offset_front);
                        int src_col_front = *(file_data_front + row_offset_front + col_offset_front+ 1);

                        if (DEBUG_bilinear) {
                            float weight[4];
                            weight[0] = *(float*)(file_data_front + row_offset_front + col_offset_front+ 2);
                            weight[1] = *(float*)(file_data_front + row_offset_front + col_offset_front+ 3);
                            weight[2] = *(float*)(file_data_front + row_offset_front + col_offset_front+ 4);
                            weight[3] = *(float*)(file_data_front + row_offset_front + col_offset_front+ 5);

                            int src_position[2];
                            src_position[0] = src_row_front;
                            src_position[1] = src_col_front;

                            int dst_position[2];
                            dst_position[0] = row;
                            dst_position[1] = col;

                            imagepixel_bilinear_generate(front_raw_image, stitch_fusion_image, src_position, dst_position, weight);
                        }
                        else{
                            stitch_fusion_image.ptr<Vec3b>(row)[col] = front_raw_image.ptr<Vec3b>(src_row_front)[src_col_front];
                        }
                    }
                }
            }
            else if(row > (left_height - rear_height))//下部分
            {
                if(col < left_width)  //下左 5
                {
                    int row_offset_left = row * left_width * 6;
                    int col_offset = col * 6;
                    int src_row_left = *(file_data_left + row_offset_left+ col_offset);
                    int src_col_left = *(file_data_left + row_offset_left + col_offset+ 1);

                    int fusion_offset_left = row * left_width + col;
                    float fusion_left = *(file_fusion_left + fusion_offset_left);

                    int row_offset_rear = (row-(left_height - rear_height)) * rear_width * 6;
                    int src_row_rear= *(file_data_rear + row_offset_rear+ col_offset);
                    int src_col_rear = *(file_data_rear + row_offset_rear + col_offset+ 1);

                    int fusion_offset_rear = (row-(left_height - rear_height)) * rear_width + col;
                    float fusion_rear = *(file_fusion_rear + fusion_offset_rear);


                    if((!rear_use)&&(!left_use))
                    {
                        fusion_rear = 0;
                        fusion_left = 0;
                    }

                    if (DEBUG_bilinear) {
                        float weight_1[4];
                        weight_1[0] = *(float*)(file_data_rear + row_offset_rear + col_offset+ 2);
                        weight_1[1] = *(float*)(file_data_rear + row_offset_rear + col_offset+ 3);
                        weight_1[2] = *(float*)(file_data_rear + row_offset_rear + col_offset+ 4);
                        weight_1[3] = *(float*)(file_data_rear + row_offset_rear + col_offset+ 5);

                        int src_position_1[2];
                        src_position_1[0] = src_row_rear;
                        src_position_1[1] = src_col_rear;

                        float weight_2[4];
                        weight_2[0] = *(float*)(file_data_left + row_offset_left+ col_offset+ 2);
                        weight_2[1] = *(float*)(file_data_left + row_offset_left+ col_offset+ 3);
                        weight_2[2] = *(float*)(file_data_left + row_offset_left+ col_offset+ 4);
                        weight_2[3] = *(float*)(file_data_left + row_offset_left+ col_offset+ 5);

                        int src_position_2[2];
                        src_position_2[0] = src_row_left;
                        src_position_2[1] = src_col_left;

                        int dst_position[2];
                        dst_position[0] = row;
                        dst_position[1] = col;

                        float fusion[2] = {0};
                        fusion[0] = fusion_rear;
                        fusion[1] = fusion_left;

                        imagepixel_bilinear_fusion(rear_raw_image, left_raw_image, stitch_fusion_image,src_position_1, dst_position, weight_1, src_position_2, weight_2, fusion);

                    }else{
                        stitch_fusion_image.ptr<Vec3b>(row)[col] = rear_raw_image.ptr<Vec3b>(src_row_rear)[src_col_rear]*fusion_rear + left_raw_image.ptr<Vec3b>(src_row_left)[src_col_left]*fusion_left;
                    }
                }
                else if(col > (rear_width - right_width))   //下右  7
                {

                    int row_offset_rear = (row-(right_height - rear_height)) * rear_width * 6;
                    int col_offset_rear = col * 6;
                    int src_row_rear= *(file_data_rear + row_offset_rear + col_offset_rear);
                    int src_col_rear = *(file_data_rear + row_offset_rear + col_offset_rear+ 1);

                    int fusion_offset_rear = (row-(right_height - rear_height)) * rear_width + col;
                    float fusion_rear = *(file_fusion_rear + fusion_offset_rear);

                    int row_offset_right = row * right_width * 6;
                    int col_offset_right = (col-(rear_width - right_width)) * 6;
                    int src_row_right = *(file_data_right + row_offset_right + col_offset_right);
                    int src_col_right = *(file_data_right + row_offset_right + col_offset_right+ 1);

                    int fusion_offset_right = row * right_width + (col-(rear_width - right_width));
                    float fusion_right = *(file_fusion_right + fusion_offset_right);

                    if((!rear_use)&&(!right_use))
                    {
                        fusion_rear = 0;
                        fusion_right = 0;
                    }

                    if (DEBUG_bilinear) {
                        float weight_1[4];
                        weight_1[0] = *(float*)(file_data_rear + row_offset_rear + col_offset_rear+ 2);
                        weight_1[1] = *(float*)(file_data_rear + row_offset_rear + col_offset_rear+ 3);
                        weight_1[2] = *(float*)(file_data_rear + row_offset_rear + col_offset_rear+ 4);
                        weight_1[3] = *(float*)(file_data_rear + row_offset_rear + col_offset_rear+ 5);

                        int src_position_1[2];
                        src_position_1[0] = src_row_rear;
                        src_position_1[1] = src_col_rear;

                        float weight_2[4];
                        weight_2[0] = *(float*)(file_data_right + row_offset_right+ col_offset_right+ 2);
                        weight_2[1] = *(float*)(file_data_right + row_offset_right+ col_offset_right+ 3);
                        weight_2[2] = *(float*)(file_data_right + row_offset_right+ col_offset_right+ 4);
                        weight_2[3] = *(float*)(file_data_right + row_offset_right+ col_offset_right+ 5);

                        int src_position_2[2];
                        src_position_2[0] = src_row_right;
                        src_position_2[1] = src_col_right;

                        int dst_position[2];
                        dst_position[0] = row;
                        dst_position[1] = col;

                        float fusion[2] = {0};
                        fusion[0] = fusion_rear;
                        fusion[1] = fusion_right;

                        imagepixel_bilinear_fusion(rear_raw_image, right_raw_image, stitch_fusion_image,src_position_1, dst_position, weight_1, src_position_2, weight_2, fusion);
                    }else{
                        stitch_fusion_image.ptr<Vec3b>(row)[col] = rear_raw_image.ptr<Vec3b>(src_row_rear)[src_col_rear] *fusion_rear + right_raw_image.ptr<Vec3b>(src_row_right)[src_col_right]*fusion_right;
                    }
                }
                else    // 6：下中间
                {
                    if(rear_use)
                    {
                        int row_offset_rear = (row-(left_height-rear_height)) * rear_width * 6;
                        int col_offset_rear = col * 6;

                        int src_row_rear= *(file_data_rear + row_offset_rear + col_offset_rear);
                        int src_col_rear = *(file_data_rear + row_offset_rear + col_offset_rear+ 1);

                        if (DEBUG_bilinear) {
                            float weight[4];
                            weight[0] = *(float*)(file_data_rear + row_offset_rear + col_offset_rear+ 2);
                            weight[1] = *(float*)(file_data_rear + row_offset_rear + col_offset_rear+ 3);
                            weight[2] = *(float*)(file_data_rear + row_offset_rear + col_offset_rear+ 4);
                            weight[3] = *(float*)(file_data_rear + row_offset_rear + col_offset_rear+ 5);

                            int src_position[2];
                            src_position[0] = src_row_rear;
                            src_position[1] = src_col_rear;

                            int dst_position[2];
                            dst_position[0] = row;
                            dst_position[1] = col;

                            imagepixel_bilinear_generate(rear_raw_image, stitch_fusion_image, src_position, dst_position, weight);
                        }else{
                            stitch_fusion_image.ptr<Vec3b>(row)[col] = rear_raw_image.ptr<Vec3b>(src_row_rear)[src_col_rear];
                        }
                    }
                }
            }
            else //中间部分
            {
                if(col < (left_width))                //左侧部分  3
                {
                    if(left_use)
                    {
                        int row_offset_left = row * left_width * 6;
                        int col_offset_left = col * 6;

                        int src_row_left = *(file_data_left + row_offset_left+ col_offset_left);
                        int src_col_left = *(file_data_left + row_offset_left + col_offset_left+ 1);

                        if (DEBUG_bilinear) {
                            float weight[4];
                            weight[0] = *(float*)(file_data_left + row_offset_left + col_offset_left+ 2);
                            weight[1] = *(float*)(file_data_left + row_offset_left + col_offset_left+ 3);
                            weight[2] = *(float*)(file_data_left + row_offset_left + col_offset_left+ 4);
                            weight[3] = *(float*)(file_data_left + row_offset_left + col_offset_left+ 5);

                            int src_position[2];
                            src_position[0] = src_row_left;
                            src_position[1] = src_col_left;

                            int dst_position[2];
                            dst_position[0] = row;
                            dst_position[1] = col;

                            imagepixel_bilinear_generate(left_raw_image, stitch_fusion_image, src_position, dst_position, weight);
                        }else{
                            stitch_fusion_image.ptr<Vec3b>(row)[col] = left_raw_image.ptr<Vec3b>(src_row_left)[src_col_left];
                        }
                    }

                }
                else if(col > (front_width - right_width))            //右侧部分4
                {
                    if(right_use) {
                        int row_offset_right = row * right_width * 6;
                        int col_offset_right = (col - (rear_width - right_width)) * 6;

                        int src_row_right = *(file_data_right + row_offset_right + col_offset_right);
                        int src_col_right = *(file_data_right + row_offset_right + col_offset_right + 1);


                        if (DEBUG_bilinear) {
                            float weight[4];
                            weight[0] = *(float *) (file_data_right + row_offset_right + col_offset_right + 2);
                            weight[1] = *(float *) (file_data_right + row_offset_right + col_offset_right + 3);
                            weight[2] = *(float *) (file_data_right + row_offset_right + col_offset_right + 4);
                            weight[3] = *(float *) (file_data_right + row_offset_right + col_offset_right + 5);

                            int src_position[2];
                            src_position[0] = src_row_right;
                            src_position[1] = src_col_right;

                            int dst_position[2];
                            dst_position[0] = row;
                            dst_position[1] = col;

                            imagepixel_bilinear_generate(right_raw_image, stitch_fusion_image, src_position,
                                                         dst_position, weight);
                        } else {
                            stitch_fusion_image.ptr<Vec3b>(row)[col] = right_raw_image.ptr<Vec3b>(src_row_right)[src_col_right];
                        }
                    }
                }
            }
        }
    }
    return stitch_fusion_image;
}


//TODO: yuv generate

Int32_t generate_stitchFusion_yuv(
        uchar* result_image,
        uchar* front_image_uyvy,
        uchar* back_image_uyvy,
        uchar* left_image_uyvy,
        uchar* right_image_uyvy, IN bool front_use, IN bool rear_use, IN bool left_use, IN bool right_use) {
    int Width = stitch_width;
    int Height = stitch_height;
    Int32_t ret = 0;
    uchar *p_src, *p_src1, *p_src2;
    memset(result_image, 0, Width * Height * 2); //内存初始化，把result_image初始化为0
    Int32_t yuv[2];
    Int32_t result_widthstep = 0, src_widthstep = 0;


    result_widthstep = stitch_width << 1;
    src_widthstep = 1280 << 1;

    for (int row = 0; row < stitch_height; ++row)
    {
        for (int col = 0; col < stitch_width; ++col)
        {
            if (row < front_height) //上部分
            {
                if (col < left_width)   //前、左融合 0
                {
                    p_src1 = front_image_uyvy;
                    p_src2 = left_image_uyvy;

                    int row_offset_front = row * front_width * 6;
                    int row_offset_left = row * left_width * 6;
                    int col_offset = col * 6;

                    int src_row_front = *(file_data_front + row_offset_front + col_offset);
                    int src_col_front = *(file_data_front + row_offset_front + col_offset + 1);
                    int src_row_left = *(file_data_left + row_offset_left + col_offset);
                    int src_col_left = *(file_data_left + row_offset_left + col_offset + 1);
                    int fusion_offset_front = row * front_width + col;
                    float fusion_front = *(file_fusion_front + fusion_offset_front);
                    int fusion_offset_left = row * left_width + col;
                    float fusion_left = *(file_fusion_left + fusion_offset_left);

                    if((!front_use)&&(!left_use))
                    {
                        fusion_front = 0;
                        fusion_left = 0;
                    }

                    int y1 = src_row_front;
                    int x1 = src_col_front;

                    int y2 = src_row_left;
                    int x2 = src_col_left;

                    x1 = (col%2 ==0) ? (x1 & 0xFFFE) : (x1 | 0x1);
                    x2 = (col%2 ==0) ? (x2 & 0xFFFE) : (x2 | 0x1);
                    if (DEBUG_bilinear) {
                        float weight_1[4];
                        weight_1[0] = *(float *) (file_data_front + row_offset_front + col_offset + 2);
                        weight_1[1] = *(float *) (file_data_front + row_offset_front + col_offset + 3);
                        weight_1[2] = *(float *) (file_data_front + row_offset_front + col_offset + 4);
                        weight_1[3] = *(float *) (file_data_front + row_offset_front + col_offset + 5);

                        yuv[0] = ((p_src1[y1 * src_widthstep + x1 * 2 + 1] *weight_1[0]      //wt 双线性插值系数 p_src1:原图；bev_Table:插值权重
                                   + p_src1[y1 * src_widthstep + (x1 + 1) * 2 + 1] * weight_1[1]
                                   + p_src1[(y1 + 1) * src_widthstep + x1 * 2 + 1] * weight_1[2]
                                   + p_src1[(y1 + 1) * src_widthstep + (x1 + 1) * 2 + 1] * weight_1[3])) * fusion_front;

                        yuv[1] = ((p_src1[y1 * src_widthstep + x1 * 2])) * fusion_front;

                        float weight_2[4];
                        weight_2[0] = *(float *) (file_data_left + row_offset_left + col_offset + 2);
                        weight_2[1] = *(float *) (file_data_left + row_offset_left + col_offset + 3);
                        weight_2[2] = *(float *) (file_data_left + row_offset_left + col_offset + 4);
                        weight_2[3] = *(float *) (file_data_left + row_offset_left + col_offset + 5);
                        yuv[0] += ((p_src2[y2 * src_widthstep + x2 * 2 + 1] * weight_2[0]
                                    + p_src2[y2 * src_widthstep + (x2 + 1) * 2 + 1] * weight_2[1]
                                    + p_src2[(y2 + 1) * src_widthstep + x2 * 2 + 1] * weight_2[2]
                                    + p_src2[(y2 + 1) * src_widthstep + (x2 + 1) * 2 + 1] * weight_2[3])) * fusion_left;

                        yuv[1] += ((p_src2[y2 * src_widthstep + x2 * 2])) * fusion_left;

                    } else {
                        yuv[0] = p_src1[y1 * src_widthstep + x1 * 2 + 1] * fusion_front +
                                 p_src2[y2 * src_widthstep + x2 * 2 + 1] * fusion_left;
                        yuv[1] = ((p_src1[y1 * src_widthstep + x1 * 2])) * fusion_front +
                                 ((p_src2[y2 * src_widthstep + x2 * 2])) * fusion_left;
                    }
                    result_image[(row)* result_widthstep + (col)* 2 + 1] += yuv[0];
                    result_image[(row)* result_widthstep + (col)* 2] += yuv[1];
                } else if (col > (front_width - right_width)) //前、右融合 2
                {

                    p_src1 = front_image_uyvy;
                    p_src2 = right_image_uyvy;

                    int row_offset_front = row * front_width * 6;
                    int col_offset_front = col * 6;
                    int row_offset_right = row * right_width * 6;
                    int col_offset_right = (col - (front_width - right_width)) * 6;

                    int src_row_front = *(file_data_front + row_offset_front + col_offset_front);
                    int src_col_front = *(file_data_front + row_offset_front + col_offset_front + 1);

                    int src_row_right = *(file_data_right + row_offset_right + col_offset_right);
                    int src_col_right = *(file_data_right + row_offset_right + col_offset_right + 1);

                    int fusion_offset_front = row * front_width + col;
                    float fusion_front = *(file_fusion_front + fusion_offset_front);

                    int fusion_offset_right = row * right_width + (col - (front_width - right_width));
                    float fusion_right = *(file_fusion_right + fusion_offset_right);

                    if((!front_use)&&(!right_use))
                    {
                        fusion_front = 0;
                        fusion_right = 0;
                    }

                    int y1 = src_row_front;
                    int x1 = src_col_front;

                    int y2 = src_row_right;
                    int x2 = src_col_right;

                    x1 = (col%2 ==0) ? (x1 & 0xFFFE) : (x1 | 0x1);
                    x2 = (col%2 ==0) ? (x2 & 0xFFFE) : (x2 | 0x1);
                    if (DEBUG_bilinear) {
                        float weight_1[4];
                        weight_1[0] = *(float *) (file_data_front + row_offset_front + col_offset_front + 2);
                        weight_1[1] = *(float *) (file_data_front + row_offset_front + col_offset_front + 3);
                        weight_1[2] = *(float *) (file_data_front + row_offset_front + col_offset_front + 4);
                        weight_1[3] = *(float *) (file_data_front + row_offset_front + col_offset_front + 5);

                        yuv[0] = ((p_src1[y1 * src_widthstep + x1 * 2 + 1] *
                                   weight_1[0]      //wt 双线性插值系数 p_src1:原图；bev_Table:插值权重
                                   + p_src1[y1 * src_widthstep + (x1 + 1) * 2 + 1] * weight_1[1]
                                   + p_src1[(y1 + 1) * src_widthstep + x1 * 2 + 1] * weight_1[2]
                                   + p_src1[(y1 + 1) * src_widthstep + (x1 + 1) * 2 + 1] * weight_1[3])) * fusion_front;

                        yuv[1] = ((p_src1[y1 * src_widthstep + x1 * 2])) * fusion_front;
                        float weight_2[4];
                        weight_2[0] = *(float *) (file_data_right + row_offset_right + col_offset_right + 2);
                        weight_2[1] = *(float *) (file_data_right + row_offset_right + col_offset_right + 3);
                        weight_2[2] = *(float *) (file_data_right + row_offset_right + col_offset_right + 4);
                        weight_2[3] = *(float *) (file_data_right + row_offset_right + col_offset_right + 5);

                        yuv[0] += ((p_src2[y2 * src_widthstep + x2 * 2 + 1] *
                                    weight_2[0]      //wt 双线性插值系数 p_src1:原图；bev_Table:插值权重
                                    + p_src2[y2 * src_widthstep + (x2 + 1) * 2 + 1] * weight_2[1]
                                    + p_src2[(y2 + 1) * src_widthstep + x2 * 2 + 1] * weight_2[2]
                                    + p_src2[(y2 + 1) * src_widthstep + (x2 + 1) * 2 + 1] * weight_2[3])) *
                                  fusion_right;
                        yuv[1] += ((p_src2[y2 * src_widthstep + x2 * 2])) * fusion_right;
                    } else {
                        yuv[0] = p_src1[y1 * src_widthstep + x1 * 2 + 1] * fusion_front +
                                 p_src2[y2 * src_widthstep + x2 * 2 + 1] * fusion_right;
                        yuv[1] = ((p_src1[y1 * src_widthstep + x1 * 2])) * fusion_front +
                                 ((p_src2[y2 * src_widthstep + x2 * 2])) * fusion_right;
                    }
                    result_image[(row)* result_widthstep + (col)* 2 + 1] += yuv[0];
                    result_image[(row)* result_widthstep + (col)* 2] += yuv[1];
                } else {        //前中间 1

                    if(front_use)
                    {
                        p_src1 = front_image_uyvy;

                        int row_offset_front = row * front_width * 6;
                        int col_offset_front = col * 6;

                        int src_row_front = *(file_data_front + row_offset_front + col_offset_front);
                        int src_col_front = *(file_data_front + row_offset_front + col_offset_front + 1);

                        int y1 = src_row_front;
                        int x1 = src_col_front;
                        x1 = (col%2 ==0) ? (x1 & 0xFFFE) : (x1 | 0x1);

                        if (DEBUG_bilinear) {
                            float weight[4];
                            weight[0] = *(float *) (file_data_front + row_offset_front + col_offset_front + 2);
                            weight[1] = *(float *) (file_data_front + row_offset_front + col_offset_front + 3);
                            weight[2] = *(float *) (file_data_front + row_offset_front + col_offset_front + 4);
                            weight[3] = *(float *) (file_data_front + row_offset_front + col_offset_front + 5);
                            yuv[0] = ((p_src1[y1 * src_widthstep + x1 * 2 + 1] * weight[0]      //wt 双线性插值系数 p_src1:原图；bev_Table:插值权重
                                       + p_src1[y1 * src_widthstep + (x1 + 1) * 2 + 1] * weight[1]
                                       + p_src1[(y1 + 1) * src_widthstep + x1 * 2 + 1] * weight[2]
                                       + p_src1[(y1 + 1) * src_widthstep + (x1 + 1) * 2 + 1] * weight[3]));
                            yuv[1] = ((p_src1[y1 * src_widthstep + x1 * 2]));

                        } else {
                            yuv[0] = p_src1[y1 * src_widthstep + x1 * 2 + 1];
                            yuv[1] = p_src1[y1 * src_widthstep + x1 * 2];
                        }
                        result_image[(row)* result_widthstep + (col)* 2 + 1] += yuv[0];
                        result_image[(row)* result_widthstep + (col)* 2] += yuv[1];
                    }
                }
            } else if (row > (left_height - rear_height))    //下部分
            {
                if (col < left_width)    //5
                {

                    p_src1 = back_image_uyvy;
                    p_src2 = left_image_uyvy;

                    int row_offset_left = row * left_width * 6;
                    int col_offset = col * 6;
                    int src_row_left = *(file_data_left + row_offset_left+ col_offset);
                    int src_col_left = *(file_data_left + row_offset_left + col_offset+ 1);

                    int fusion_offset_left = row * left_width + col;
                    float fusion_left = *(file_fusion_left + fusion_offset_left);

                    int row_offset_rear = (row-(left_height - rear_height)) * rear_width * 6;
                    int src_row_rear= *(file_data_rear + row_offset_rear+ col_offset);
                    int src_col_rear = *(file_data_rear + row_offset_rear + col_offset+ 1);

                    int fusion_offset_rear = (row-(left_height - rear_height)) * rear_width + col;
                    float fusion_rear = *(file_fusion_rear + fusion_offset_rear);
                    if((!rear_use)&&(!left_use))
                    {
                        fusion_rear = 0;
                        fusion_left = 0;
                    }
                    int y1 = src_row_rear;
                    int x1 = src_col_rear;

                    int y2 = src_row_left;
                    int x2 = src_col_left;
                    x1 = (col%2 ==0) ? (x1 & 0xFFFE) : (x1 | 0x1);
                    x2 = (col%2 ==0) ? (x2 & 0xFFFE) : (x2 | 0x1);
                    if (DEBUG_bilinear) {
                        float weight_1[4];
                        weight_1[0] = *(float*)(file_data_rear + row_offset_rear + col_offset+ 2);
                        weight_1[1] = *(float*)(file_data_rear + row_offset_rear + col_offset+ 3);
                        weight_1[2] = *(float*)(file_data_rear + row_offset_rear + col_offset+ 4);
                        weight_1[3] = *(float*)(file_data_rear + row_offset_rear + col_offset+ 5);

                        yuv[0] = ((p_src1[y1 * src_widthstep + x1 * 2 + 1] * weight_1[0]      //wt 双线性插值系数 p_src1:原图；bev_Table:插值权重
                                   + p_src1[y1 * src_widthstep + (x1 + 1) * 2 + 1] * weight_1[1]
                                   + p_src1[(y1 + 1) * src_widthstep + x1 * 2 + 1] * weight_1[2]
                                   + p_src1[(y1 + 1) * src_widthstep + (x1 + 1) * 2 + 1] * weight_1[3])) * fusion_rear;
                        yuv[1] = ((p_src1[y1 * src_widthstep + x1 * 2])) * fusion_rear;

                        float weight_2[4];
                        weight_2[0] = *(float*)(file_data_left + row_offset_left+ col_offset+ 2);
                        weight_2[1] = *(float*)(file_data_left + row_offset_left+ col_offset+ 3);
                        weight_2[2] = *(float*)(file_data_left + row_offset_left+ col_offset+ 4);
                        weight_2[3] = *(float*)(file_data_left + row_offset_left+ col_offset+ 5);
                        yuv[0] += ((p_src2[y2 * src_widthstep + x2 * 2 + 1] * weight_2[0]      //wt 双线性插值系数 p_src1:原图；bev_Table:插值权重
                                    + p_src2[y2 * src_widthstep + (x2 + 1) * 2 + 1] * weight_2[1]
                                    + p_src2[(y2 + 1) * src_widthstep + x2 * 2 + 1] * weight_2[2]
                                    + p_src2[(y2 + 1) * src_widthstep + (x2 + 1) * 2 + 1] * weight_2[3])) *
                                  fusion_left;
                        yuv[1] += ((p_src2[y2 * src_widthstep + x2 * 2])) * fusion_left;
                    }else{
                        yuv[0] = p_src1[y1 * src_widthstep + x1 * 2 + 1] * fusion_rear +
                                 p_src2[y2 * src_widthstep + x2 * 2 + 1] * fusion_left;
                        yuv[1] = ((p_src1[y1 * src_widthstep + x1 * 2])) * fusion_rear +
                                 ((p_src2[y2 * src_widthstep + x2 * 2])) * fusion_left;
                    }
                    result_image[(row)* result_widthstep + (col)* 2 + 1] += yuv[0];
                    result_image[(row)* result_widthstep + (col)* 2] += yuv[1];
                } else if (col > (rear_width - right_width))   // 7
                {
                    p_src1 = back_image_uyvy;
                    p_src2 = right_image_uyvy;

                    int row_offset_rear = (row-(right_height - rear_height)) * rear_width * 6;
                    int col_offset_rear = col * 6;
                    int src_row_rear= *(file_data_rear + row_offset_rear + col_offset_rear);
                    int src_col_rear = *(file_data_rear + row_offset_rear + col_offset_rear+ 1);

                    int fusion_offset_rear = (row-(right_height - rear_height)) * rear_width + col;
                    float fusion_rear = *(file_fusion_rear + fusion_offset_rear);

                    int row_offset_right = row * right_width * 6;
                    int col_offset_right = (col-(rear_width - right_width)) * 6;
                    int src_row_right = *(file_data_right + row_offset_right + col_offset_right);
                    int src_col_right = *(file_data_right + row_offset_right + col_offset_right+ 1);

                    int fusion_offset_right = row * right_width + (col-(rear_width - right_width));
                    float fusion_right = *(file_fusion_right + fusion_offset_right);

                    if((!rear_use)&&(!right_use))
                    {
                        fusion_rear = 0;
                        fusion_right = 0;
                    }

                    int y1 = src_row_rear;
                    int x1 = src_col_rear;

                    int y2 = src_row_right;
                    int x2 = src_col_right;
                    x1 = (col%2 ==0) ? (x1 & 0xFFFE) : (x1 | 0x1);
                    x2 = (col%2 ==0) ? (x2 & 0xFFFE) : (x2 | 0x1);
                    if (DEBUG_bilinear) {
                        float weight_1[4];
                        weight_1[0] = *(float*)(file_data_rear + row_offset_rear + col_offset_rear+ 2);
                        weight_1[1] = *(float*)(file_data_rear + row_offset_rear + col_offset_rear+ 3);
                        weight_1[2] = *(float*)(file_data_rear + row_offset_rear + col_offset_rear+ 4);
                        weight_1[3] = *(float*)(file_data_rear + row_offset_rear + col_offset_rear+ 5);

                        yuv[0] = ((p_src1[y1 * src_widthstep + x1 * 2 + 1] * weight_1[0]      //wt 双线性插值系数 p_src1:原图；bev_Table:插值权重
                                   + p_src1[y1 * src_widthstep + (x1 + 1) * 2 + 1] * weight_1[1]
                                   + p_src1[(y1 + 1) * src_widthstep + x1 * 2 + 1] * weight_1[2]
                                   + p_src1[(y1 + 1) * src_widthstep + (x1 + 1) * 2 + 1] * weight_1[3])) * fusion_rear;
                        yuv[1] = ((p_src1[y1 * src_widthstep + x1 * 2])) * fusion_rear;

                        float weight_2[4];
                        weight_2[0] = *(float*)(file_data_right + row_offset_right+ col_offset_right+ 2);
                        weight_2[1] = *(float*)(file_data_right + row_offset_right+ col_offset_right+ 3);
                        weight_2[2] = *(float*)(file_data_right + row_offset_right+ col_offset_right+ 4);
                        weight_2[3] = *(float*)(file_data_right + row_offset_right+ col_offset_right+ 5);

                        yuv[0] += ((p_src2[y2 * src_widthstep + x2 * 2 + 1] * weight_2[0]      //wt 双线性插值系数 p_src1:原图；bev_Table:插值权重
                                    + p_src2[y2 * src_widthstep + (x2 + 1) * 2 + 1] * weight_2[1]
                                    + p_src2[(y2 + 1) * src_widthstep + x2 * 2 + 1] * weight_2[2]
                                    + p_src2[(y2 + 1) * src_widthstep + (x2 + 1) * 2 + 1] * weight_2[3])) *
                                  fusion_right;
                        yuv[1] += ((p_src2[y2 * src_widthstep + x2 * 2])) * fusion_right;

                    }else{
                        yuv[0] = p_src1[y1 * src_widthstep + x1 * 2 + 1] * fusion_rear +
                                 p_src2[y2 * src_widthstep + x2 * 2 + 1] * fusion_right;
                        yuv[1] = ((p_src1[y1 * src_widthstep + x1 * 2])) * fusion_rear +
                                 ((p_src2[y2 * src_widthstep + x2 * 2])) * fusion_right;
                       }
                    result_image[(row)* result_widthstep + (col)* 2 + 1] += yuv[0];
                    result_image[(row)* result_widthstep + (col)* 2] += yuv[1];
                } else    //  下中间 6
                {
                    if(rear_use)
                    {
                        p_src1 = back_image_uyvy;

                        int row_offset_rear = (row-(left_height-rear_height)) * rear_width * 6;
                        int col_offset_rear = col * 6;

                        int src_row_rear= *(file_data_rear + row_offset_rear + col_offset_rear);
                        int src_col_rear = *(file_data_rear + row_offset_rear + col_offset_rear+ 1);

                        int y1 = src_row_rear;
                        int x1 = src_col_rear;
                        x1 = (col%2 ==0) ? (x1 & 0xFFFE) : (x1 | 0x1);
                        if (DEBUG_bilinear) {
                            float weight[4];
                            weight[0] = *(float*)(file_data_rear + row_offset_rear + col_offset_rear+ 2);
                            weight[1] = *(float*)(file_data_rear + row_offset_rear + col_offset_rear+ 3);
                            weight[2] = *(float*)(file_data_rear + row_offset_rear + col_offset_rear+ 4);
                            weight[3] = *(float*)(file_data_rear + row_offset_rear + col_offset_rear+ 5);

                            yuv[0] = ((p_src1[y1 * src_widthstep + x1 * 2 + 1] * weight[0]      //wt 双线性插值系数 p_src1:原图；bev_Table:插值权重
                                       + p_src1[y1 * src_widthstep + (x1 + 1) * 2 + 1] * weight[1]
                                       + p_src1[(y1 + 1) * src_widthstep + x1 * 2 + 1] * weight[2]
                                       + p_src1[(y1 + 1) * src_widthstep + (x1 + 1) * 2 + 1] * weight[3]));
                            yuv[1] = ((p_src1[y1 * src_widthstep + x1 * 2]));
                        }else{
                            yuv[0] = p_src1[y1 * src_widthstep + x1 * 2 + 1];
                            yuv[1] = p_src1[y1 * src_widthstep + x1 * 2];

                        }
                        result_image[(row)* result_widthstep + (col)* 2 + 1] += yuv[0];
                        result_image[(row)* result_widthstep + (col)* 2] += yuv[1];
                    }
                }

            } else   //中间部分
            {
                if (col < left_width)    //3
                {
                    if(left_use)
                    {
                        p_src1 = left_image_uyvy;

                        int row_offset_left = row * left_width * 6;
                        int col_offset_left = col * 6;

                        int src_row_left = *(file_data_left + row_offset_left+ col_offset_left);
                        int src_col_left = *(file_data_left + row_offset_left + col_offset_left+ 1);

                        int y1 = src_row_left;
                        int x1 = src_col_left;
                        x1 = (col%2 ==0) ? (x1 & 0xFFFE) : (x1 | 0x1);
                        if (DEBUG_bilinear) {
                            float weight[4];
                            weight[0] = *(float*)(file_data_left + row_offset_left + col_offset_left+ 2);
                            weight[1] = *(float*)(file_data_left + row_offset_left + col_offset_left+ 3);
                            weight[2] = *(float*)(file_data_left + row_offset_left + col_offset_left+ 4);
                            weight[3] = *(float*)(file_data_left + row_offset_left + col_offset_left+ 5);

                            yuv[0] = ((p_src1[y1 * src_widthstep + x1 * 2 + 1] * weight[0]      //wt 双线性插值系数 p_src1:原图；bev_Table:插值权重
                                       + p_src1[y1 * src_widthstep + (x1 + 1) * 2 + 1] * weight[1]
                                       + p_src1[(y1 + 1) * src_widthstep + x1 * 2 + 1] * weight[2]
                                       + p_src1[(y1 + 1) * src_widthstep + (x1 + 1) * 2 + 1] * weight[3]));
                            yuv[1] = ((p_src1[y1 * src_widthstep + x1 * 2]));
                        }else{
                            yuv[0] = p_src1[y1 * src_widthstep + x1 * 2 + 1];
                            yuv[1] = p_src1[y1 * src_widthstep + x1 * 2];
                        }
                        result_image[(row)* result_widthstep + (col)* 2 + 1] = yuv[0];
                        result_image[(row)* result_widthstep + (col)* 2] = yuv[1];
                    }
                } else if (col > (rear_width - right_width))    //4
                {
                    if(right_use)
                    {
                        p_src1 = right_image_uyvy;

                        int row_offset_right = row * right_width * 6;
                        int col_offset_right = (col - (rear_width - right_width)) * 6;

                        int src_row_right = *(file_data_right + row_offset_right + col_offset_right);
                        int src_col_right = *(file_data_right + row_offset_right + col_offset_right + 1);

                        int y1 = src_row_right;
                        int x1 = src_col_right;
                        x1 = (col%2 ==0) ? (x1 & 0xFFFE) : (x1 | 0x1);

                        if (DEBUG_bilinear) {
                            float weight[4];
                            weight[0] = *(float *) (file_data_right + row_offset_right + col_offset_right + 2);
                            weight[1] = *(float *) (file_data_right + row_offset_right + col_offset_right + 3);
                            weight[2] = *(float *) (file_data_right + row_offset_right + col_offset_right + 4);
                            weight[3] = *(float *) (file_data_right + row_offset_right + col_offset_right + 5);

                            yuv[0] = ((p_src1[y1 * src_widthstep + x1 * 2 + 1] * weight[0]      //wt 双线性插值系数 p_src1:原图；bev_Table:插值权重
                                       + p_src1[y1 * src_widthstep + (x1 + 1) * 2 + 1] * weight[1]
                                       + p_src1[(y1 + 1) * src_widthstep + x1 * 2 + 1] * weight[2]
                                       + p_src1[(y1 + 1) * src_widthstep + (x1 + 1) * 2 + 1] * weight[3]));
                            yuv[1] = ((p_src1[y1 * src_widthstep + x1 * 2]));

                        } else {
                            yuv[0] = p_src1[y1 * src_widthstep + x1 * 2 + 1];
                            yuv[1] = p_src1[y1 * src_widthstep + x1 * 2];
                        }
                        result_image[(row)* result_widthstep + (col)* 2 + 1] = yuv[0];
                        result_image[(row)* result_widthstep + (col)* 2] = yuv[1];
                    }
                }
            }
        }
    }
}

/*
 * author: liuli
 * date:
 * name:image_addGain
 * function: 输入融合单路图，bgr2yuv + gain + yuv2bgr 输出加上增益后到图
 * parametes:
 *          INOUT： Mat stitch_raw_image 原始鱼眼图
 *          IN:     VIEW_E view_index
                    view_front_e,
                    view_rear_e,
                    view_left_e,
                    view_right_e,
 *
 *
 *
 *return;
 * */
int image_addGain(INOUT Mat stitch_raw_image, IN VIEW_E view_index)
{
    int rtn = 0;
    int gain_index = 0;
    if (view_front_e  == view_index)
        gain_index = 0;
    if (view_rear_e  == view_index)
        gain_index = 1;
    if (view_left_e  == view_index)
        gain_index = 2;
    if (view_right_e  == view_index)
        gain_index = 3;

    int col = stitch_raw_image.cols;
    int row = stitch_raw_image.rows;
    float bgr[3] = {0};
    float yuv[3] = {0};

    for (int i = 0; i < col; i++)
        for (int j = 0; j < row; j++)
        {
            bgr[0] = stitch_raw_image.ptr<Vec3b>(j)[i][0] ;
            bgr[1] = stitch_raw_image.ptr<Vec3b>(j)[i][1] ;
            bgr[2] = stitch_raw_image.ptr<Vec3b>(j)[i][2] ;

            bgrToYuv(bgr, yuv);
            yuv[0] += currentGains[gain_index];
            yuv[1] += currentGains[gain_index + 4];
            yuv[2] += currentGains[gain_index + 8];
            yuvToBgr(yuv, bgr);
            stitch_raw_image.ptr<Vec3b>(j)[i][0] = bgr[0] ;
            stitch_raw_image.ptr<Vec3b>(j)[i][1] = bgr[1] ;
            stitch_raw_image.ptr<Vec3b>(j)[i][2] = bgr[2] ;
        }

    return rtn;
}


/*
 * author: cheng
 * date:
 * name:lutTable_generate_image
 * function: 使用lut table，生成相应的效果图
 * parametes:
 *          IN： Mat raw_image 原始鱼眼图
 *          OUT
 *        INOUT： Mat dst_image 使用lut表生成的图
 *
 *return;
 * */
int lutTable_generate_image(IN Mat raw_image, INOUT Mat dst_image, IN IMAGE_VIEW_E image_view_e, IN VIEW_E view_index, IN FUSION_E stitch_fusion)
{
    int rtn = 0;
    int dst_width = dst_image.cols;
    int dst_height= dst_image.rows;

    uint32_t src_row = 0;
    uint32_t src_col = 0;
    float a1 = 0;
    float a2 = 0;
    float a3 = 0;
    float a4 = 0;

    int src_position[2] = {0};
    int dst_position[2] = {0};
    float weight[4] = {0};
    FILE* fp_lut;
    FILE* fp_fusion;


    if (image_view_projective_e == image_view_e)
    {
        if(view_front_e == view_index)
        {
            string lut_file_name = run_dir + "projective_front.bin";
            fp_lut = fopen(lut_file_name.c_str(),"rb");
        }
        else if(view_rear_e == view_index)
        {
            string lut_file_name = run_dir + "projective_rear.bin";
            fp_lut = fopen(lut_file_name.c_str(),"rb");
        }
        else if(view_left_e == view_index)
        {
            string lut_file_name = run_dir + "projective_left.bin";
            fp_lut = fopen(lut_file_name.c_str(),"rb");
        }
        else if(view_right_e == view_index)
        {
            string lut_file_name = run_dir + "projective_right.bin";
            fp_lut = fopen(lut_file_name.c_str(),"rb");
        }
    }
    else if(image_view_birdview_e == image_view_e)
    {
        if(view_front_e == view_index)
        {
            string lut_file_name = run_dir + "birdview_front.bin";
            fp_lut = fopen(lut_file_name.c_str(),"rb");
        }
        else if(view_rear_e == view_index)
        {
            string lut_file_name = run_dir + "birdview_rear.bin";
            fp_lut = fopen(lut_file_name.c_str(),"rb");
        }
        else if(view_left_e == view_index)
        {
            string lut_file_name = run_dir + "birdview_left.bin";
            fp_lut = fopen(lut_file_name.c_str(),"rb");
        }
        else if(view_right_e == view_index)
        {
            string lut_file_name = run_dir + "birdview_right.bin";
            fp_lut = fopen(lut_file_name.c_str(),"rb");
        }
    }
    else if(image_view_stitch_e == image_view_e)
    {

        if(view_front_e == view_index)
        {
            string lut_file_name = run_dir + "birdview_front.bin";
            fp_lut = fopen(lut_file_name.c_str(),"rb");

            string fusion_file_name = run_dir + "fusion_front.bin";
            fp_fusion = fopen(fusion_file_name.c_str(),"rb");
        }
        else if(view_rear_e == view_index)
        {
            string lut_file_name = run_dir + "birdview_rear.bin";
            fp_lut = fopen(lut_file_name.c_str(),"rb");

            string fusion_file_name = run_dir + "fusion_rear.bin";
            fp_fusion = fopen(fusion_file_name.c_str(),"rb");
        }
        else if(view_left_e == view_index)
        {
            string lut_file_name = run_dir + "birdview_left.bin";
            fp_lut = fopen(lut_file_name.c_str(),"rb");

            string fusion_file_name = run_dir + "fusion_left.bin";
            fp_fusion = fopen(fusion_file_name.c_str(),"rb");
        }
        else if(view_right_e == view_index)
        {
            string lut_file_name = run_dir + "birdview_right.bin";
            fp_lut = fopen(lut_file_name.c_str(),"rb");

            string fusion_file_name = run_dir + "fusion_right.bin";
            fp_fusion = fopen(fusion_file_name.c_str(),"rb");
        }
    }
    else if(image_view_multiview_e == image_view_e)
    {
        string lut_file_name = run_dir + "multiview.bin";
        fp_lut = fopen(lut_file_name.c_str(),"rb");
    }
    else
    {
        printf(" error the wrong lut table file,please check it !\n");
        return -1;
    }

    uint32_t *file_data = NULL;
    float *file_fusion = NULL;

    if (image_view_stitch_e == image_view_e ) //用于俯视融合图的生成，根据不同的融合输入，判断融合区是否需要融合
    {
        if(view_front_e == view_index || view_rear_e == view_index)
        {
            float fusion = 0.0;

            if(stitch_fusion == fusion_left_right_e)
            {

                file_data = (uint32_t*)malloc(dst_height*dst_width*(sizeof(uint32_t)*2 + 4*sizeof(float)));
                fread(file_data, dst_height*dst_width*(sizeof(uint32_t)*2 + 4*sizeof(float)),1,fp_lut);

                file_fusion = (float*)malloc(front_height * front_width * sizeof(float));
                fread(file_fusion, front_height * front_width * sizeof(float), 1, fp_fusion);

                int fusion_offset = 0;

                for(int row = 0; row < dst_height; row++)
                {
                    dst_position[0] = row;

                    int row_offset = row * dst_width * 6;

                    for (int col = 0; col < dst_width; col++)
                    {

                        dst_position[1] = col;

                        int col_offset =  col*6;

                        src_row = *(file_data + row_offset+ col_offset);
                        src_col = *(file_data +row_offset + col_offset+ 1);

                        fusion = *(file_fusion + fusion_offset);
                        fusion_offset++;

                        if (DEBUG_bilinear)
                        {
                            a1       = *(float*)(file_data + row_offset+col_offset+ 2);
                            a2       = *(float*)(file_data + row_offset+col_offset+ 3);
                            a3       = *(float*)(file_data + row_offset+col_offset+ 4);
                            a4       = *(float*)(file_data + row_offset+col_offset+ 5);

                            weight[0] = a1;
                            weight[1] = a2;
                            weight[2] = a3;
                            weight[3] = a4;

                            src_position[0] = src_row;
                            src_position[1] = src_col;

                            weight[0] = weight[0] * fusion;
                            weight[1] = weight[1] * fusion;
                            weight[2] = weight[2] * fusion;
                            weight[3] = weight[3] * fusion;


                        imagepixel_bilinear_generate(raw_image, dst_image, src_position, dst_position, weight);
                        }
                        else
                        {
                            if(dst_image.channels() == 3)
                                dst_image.ptr<Vec3b>(row)[col] =
                                        raw_image.ptr<Vec3b>(src_row)[src_col] * fusion;
                            else if(dst_image.channels() == 1)
                                dst_image.ptr<uchar>(row)[col] =
                                        raw_image.ptr<uchar>(src_row)[src_col] * fusion;
                        }
                    }
                }
            }
            else if(stitch_fusion == fusion_left_e)
            {

                file_data = (uint32_t*)malloc(dst_height*dst_width*(sizeof(int)*2 + 4*sizeof(float)));
                fread(file_data,dst_height*dst_width*(sizeof(int)*2 + 4*sizeof(float)),1,fp_lut);

                file_fusion = (float*)malloc(front_height * front_width * sizeof(float));
                fread(file_fusion, front_height * front_width * sizeof(float), 1, fp_fusion);
                int fusion_offset = 0;

                for(int row = 0; row < dst_height; row++)
                {
                    dst_position[0] = row;
                    int row_offset = row * dst_width * 6;
                    for (int col = 0; col < dst_width; col++)
                    {
                        dst_position[1] = col;
                        int col_offset =  col*6;

                        src_row = *(file_data + row_offset+ col_offset);
                        src_col = *(file_data +row_offset + col_offset+ 1);


                        fusion = *(file_fusion + fusion_offset);
                        fusion_offset++;

                        if (DEBUG_bilinear)
                        {
                            a1       = *(float*)(file_data + row_offset+col_offset+ 2);
                            a2       = *(float*)(file_data + row_offset+col_offset+ 3);
                            a3       = *(float*)(file_data + row_offset+col_offset+ 4);
                            a4       = *(float*)(file_data + row_offset+col_offset+ 5);

                            weight[0] = a1;
                            weight[1] = a2;
                            weight[2] = a3;
                            weight[3] = a4;

                            src_position[0] = src_row;
                            src_position[1] = src_col;

                            weight[0] = weight[0] * fusion;
                            weight[1] = weight[1] * fusion;
                            weight[2] = weight[2] * fusion;
                            weight[3] = weight[3] * fusion;


                            imagepixel_bilinear_generate(raw_image, dst_image, src_position, dst_position, weight);
                        }
                        else
                        {
                            if(dst_image.channels() == 3)
                                dst_image.ptr<Vec3b>(row)[col] =
                                    raw_image.ptr<Vec3b>(src_row)[src_col] * fusion;
                            else if(dst_image.channels() == 1)
                                dst_image.ptr<uchar>(row)[col] =
                                        raw_image.ptr<uchar>(src_row)[src_col] * fusion;

                        }
                    }
                }
            }
            else if(stitch_fusion == fusion_right_e)
            {
                file_data = (uint32_t*)malloc(dst_height*dst_width*(sizeof(int)*2 + 4*sizeof(float)));
                fread(file_data,dst_height*dst_width*(sizeof(int)*2 + 4*sizeof(float)),1,fp_lut);

                file_fusion = (float*)malloc(front_height * front_width * sizeof(float));
                fread(file_fusion, front_height * front_width * sizeof(float), 1, fp_fusion);
                int fusion_offset = 0;

                for(int row = 0; row < dst_height; row++) {
                    dst_position[0] = row;
                    int row_offset = row * dst_width * 6;
                    for (int col = 0; col < dst_width; col++) {
                        dst_position[1] = col;
                        int col_offset =  col*6;

                        src_row = *(file_data + row_offset+ col_offset);
                        src_col = *(file_data +row_offset + col_offset+ 1);

                        fusion = *(file_fusion + fusion_offset);
                        fusion_offset++;

                        if (DEBUG_bilinear)
                        {
                            a1       = *(float*)(file_data + row_offset+col_offset+ 2);
                            a2       = *(float*)(file_data + row_offset+col_offset+ 3);
                            a3       = *(float*)(file_data + row_offset+col_offset+ 4);
                            a4       = *(float*)(file_data + row_offset+col_offset+ 5);

                            weight[0] = a1;
                            weight[1] = a2;
                            weight[2] = a3;
                            weight[3] = a4;

                            src_position[0] = src_row;
                            src_position[1] = src_col;

                            weight[0] = weight[0] * fusion;
                            weight[1] = weight[1] * fusion;
                            weight[2] = weight[2] * fusion;
                            weight[3] = weight[3] * fusion;


                            imagepixel_bilinear_generate(raw_image, dst_image, src_position, dst_position, weight);
                        }
                        else
                        {
                            if(dst_image.channels() == 3)
                                dst_image.ptr<Vec3b>(row)[col] =
                                        raw_image.ptr<Vec3b>(src_row)[src_col] * fusion;
                            else if(dst_image.channels() == 1)
                                dst_image.ptr<uchar>(row)[col] =
                                        raw_image.ptr<uchar>(src_row)[src_col] * fusion;
                        }
                    }
                }
            }
        }

        else if(view_left_e == view_index || view_right_e == view_index)
        {
            float fusion = 0.0;

            if(stitch_fusion == fusion_front_rear_e)
            {
                file_data = (uint32_t*)malloc(dst_height*dst_width*(sizeof(int)*2 + 4*sizeof(float)));
                fread(file_data,dst_height*dst_width*(sizeof(int)*2 + 4*sizeof(float)),1,fp_lut);

                file_fusion = (float*)malloc(left_height * left_width * sizeof(float));
                fread(file_fusion, left_height * left_width * sizeof(float), 1, fp_fusion);
                int fusion_offset = 0;

                for(int row = 0; row < dst_height; row++)
                {
                    dst_position[0] = row;
                    int row_offset = row * dst_width * 6;
                    for (int col = 0; col < dst_width; col++)
                    {

                        dst_position[1] = col;
                        int col_offset =  col*6;

                        src_row = *(file_data + row_offset+ col_offset);
                        src_col = *(file_data +row_offset + col_offset+ 1);

                        fusion = *(file_fusion + fusion_offset);
                        fusion_offset++;

                        if (DEBUG_bilinear)
                        {
                            a1       = *(float*)(file_data + row_offset+col_offset+ 2);
                            a2       = *(float*)(file_data + row_offset+col_offset+ 3);
                            a3       = *(float*)(file_data + row_offset+col_offset+ 4);
                            a4       = *(float*)(file_data + row_offset+col_offset+ 5);

                            weight[0] = a1;
                            weight[1] = a2;
                            weight[2] = a3;
                            weight[3] = a4;

                            src_position[0] = src_row;
                            src_position[1] = src_col;

                            weight[0] = weight[0] * fusion;
                            weight[1] = weight[1] * fusion;
                            weight[2] = weight[2] * fusion;
                            weight[3] = weight[3] * fusion;

                            imagepixel_bilinear_generate(raw_image, dst_image, src_position, dst_position, weight);
                        }
                        else
                        {
                            if(dst_image.channels() == 3)
                                dst_image.ptr<Vec3b>(row)[col] =
                                        raw_image.ptr<Vec3b>(src_row)[src_col] * fusion;
                            else if(dst_image.channels() == 1)
                                dst_image.ptr<uchar>(row)[col] =
                                        raw_image.ptr<uchar>(src_row)[src_col] * fusion;
                        }
                    }
                }
            }
            else if(stitch_fusion == fusion_front_e)
            {
                file_data = (uint32_t*)malloc(dst_height*dst_width*(sizeof(int)*2 + 4*sizeof(float)));
                fread(file_data,dst_height*dst_width*(sizeof(int)*2 + 4*sizeof(float)),1,fp_lut);

                file_fusion = (float*)malloc(left_height * left_width * sizeof(float));
                fread(file_fusion, left_height * left_width * sizeof(float), 1, fp_fusion);
                int fusion_offset = 0;

                for(int row = 0; row < dst_height; row++)
                {
                    dst_position[0] = row;
                    int row_offset = row * dst_width * 6;
                    for (int col = 0; col < dst_width; col++)
                    {
                        dst_position[1] = col;
                        int col_offset =  col*6;

                        src_row = *(file_data + row_offset+ col_offset);
                        src_col = *(file_data +row_offset + col_offset+ 1);

                        fusion = *(file_fusion + fusion_offset);
                        fusion_offset++;

                        if (DEBUG_bilinear)
                        {
                            a1       = *(float*)(file_data + row_offset+col_offset+ 2);
                            a2       = *(float*)(file_data + row_offset+col_offset+ 3);
                            a3       = *(float*)(file_data + row_offset+col_offset+ 4);
                            a4       = *(float*)(file_data + row_offset+col_offset+ 5);

                            weight[0] = a1;
                            weight[1] = a2;
                            weight[2] = a3;
                            weight[3] = a4;

                            src_position[0] = src_row;
                            src_position[1] = src_col;

                            weight[0] = weight[0] * fusion;
                            weight[1] = weight[1] * fusion;
                            weight[2] = weight[2] * fusion;
                            weight[3] = weight[3] * fusion;

                            imagepixel_bilinear_generate(raw_image, dst_image, src_position, dst_position, weight);
                        }
                        else
                        {
                            if(dst_image.channels() == 3)
                                dst_image.ptr<Vec3b>(row)[col] =
                                        raw_image.ptr<Vec3b>(src_row)[src_col] * fusion;
                            else if(dst_image.channels() == 1)
                                dst_image.ptr<uchar>(row)[col] =
                                        raw_image.ptr<uchar>(src_row)[src_col] * fusion;
                        }
                    }
                }
            }
            else if(stitch_fusion == fusion_rear_e)
            {

                file_data = (uint32_t*)malloc(dst_height*dst_width*(sizeof(int)*2 + 4*sizeof(float)));
                fread(file_data,dst_height*dst_width*(sizeof(int)*2 + 4*sizeof(float)),1,fp_lut);

                file_fusion = (float*)malloc(left_height * left_width * sizeof(float));
                fread(file_fusion, left_height * left_width * sizeof(float), 1, fp_fusion);
                int fusion_offset = 0;

                for(int row = 0; row < dst_height; row++)
                {
                    dst_position[0] = row;
                    int row_offset = row * dst_width * 6;

                    for (int col = 0; col < dst_width; col++)
                    {
                        dst_position[1] = col;
                        int col_offset =  col*6;

                        src_row = *(file_data + row_offset+ col_offset);
                        src_col = *(file_data +row_offset + col_offset+ 1);

                        fusion = *(file_fusion + fusion_offset);
                        fusion_offset++;

                        if (DEBUG_bilinear)
                        {
                            a1       = *(float*)(file_data + row_offset+col_offset+ 2);
                            a2       = *(float*)(file_data + row_offset+col_offset+ 3);
                            a3       = *(float*)(file_data + row_offset+col_offset+ 4);
                            a4       = *(float*)(file_data + row_offset+col_offset+ 5);

                            weight[0] = a1;
                            weight[1] = a2;
                            weight[2] = a3;
                            weight[3] = a4;

                            src_position[0] = src_row;
                            src_position[1] = src_col;

                            weight[0] = weight[0] * fusion;
                            weight[1] = weight[1] * fusion;
                            weight[2] = weight[2] * fusion;
                            weight[3] = weight[3] * fusion;


                            imagepixel_bilinear_generate(raw_image, dst_image, src_position, dst_position, weight);
                        }
                        else
                        {
                            if(dst_image.channels() == 3)
                                dst_image.ptr<Vec3b>(row)[col] =
                                        raw_image.ptr<Vec3b>(src_row)[src_col] * fusion;
                            else if(dst_image.channels() == 1)
                                dst_image.ptr<uchar>(row)[col] =
                                        raw_image.ptr<uchar>(src_row)[src_col] * fusion;
                        }
                    }
                }
            }
        }
        free(file_fusion);
        file_fusion = NULL;

        fclose(fp_fusion);
    }
    else //非俯视、融合区的生成
    {
        file_data = (uint32_t*)malloc(dst_height*dst_width*(sizeof(int)*2 + 4*sizeof(float)));
        fread(file_data,dst_height*dst_width*(sizeof(int)*2 + 4*sizeof(float)),1,fp_lut);

        for(int row = 0; row < dst_height; row++)
        {
            int row_offset = row * dst_width * 6;
            dst_position[0] = row;
            for (int col = 0; col < dst_width; col++)
            {
                dst_position[1] = col;
                int col_offset =  col*6;

                src_row = *(file_data + row_offset+ col_offset);
                src_col = *(file_data +row_offset + col_offset+ 1);

                if (DEBUG_bilinear)
                {
                    a1       = *(float*)(file_data + row_offset+col_offset+ 2);
                    a2       = *(float*)(file_data + row_offset+col_offset+ 3);
                    a3       = *(float*)(file_data + row_offset+col_offset+ 4);
                    a4       = *(float*)(file_data + row_offset+col_offset+ 5);

                    weight[0] = a1;
                    weight[1] = a2;
                    weight[2] = a3;
                    weight[3] = a4;

                    src_position[0] = src_row;
                    src_position[1] = src_col;

                    imagepixel_bilinear_generate(raw_image, dst_image, src_position, dst_position, weight);
                }
                else
                {
                    if(dst_image.channels() == 3)
                        dst_image.ptr<Vec3b>(row)[col] =
                                raw_image.ptr<Vec3b>(src_row)[src_col];
                    else if(dst_image.channels() == 1)
                        dst_image.ptr<uchar>(row)[col] =
                                raw_image.ptr<uchar>(src_row)[src_col];
                }
            }
        }
    }

    fclose(fp_lut);

    if(file_data != NULL)
    {
        free(file_data);
        file_data = NULL;
    }

    return  rtn;

}



/*
 * author: cheng
 * date:
 * name:birdview_image_stitch
 * function: 生成一张包含前、后、左、右相机俯视图的非融合拼接图
 * parametes:
 *          IN： Mat front_image 前原始鱼眼图
 *          IN： Mat rear_image 后原始鱼眼图
 *          IN： Mat left_image 左原始鱼眼图
 *          IN： Mat right_image 右原始鱼眼图
 *          OUT：Mat dst_image 最终的拼接图
 *        INOUT：
 *
 *return;
 * */
Mat birdview_image_stitch(IN Mat front_image, IN Mat rear_image, IN Mat left_image, IN Mat right_image)
{
    int front_height = 0;
    int front_width = 0;
    int rear_height = 0;
    int rear_width = 0;
    int left_height = 0;
    int left_width = 0;
    int right_width = 0;
    int right_height = 0;

    char s[100];
    //  FILE* f_lut = fopen("./lut/birdview_size.txt","r");

    string file_name = run_dir + "birdview_size.txt";
    FILE* f_lut = fopen(file_name.c_str(),"r");

    fscanf(f_lut,"front_pixel_height:%d   ",&front_height);
    fscanf(f_lut,"front_pixel_width:%d\n",&front_width);

    fscanf(f_lut,"rear_pixel_height:%d   ",&rear_height);
    fscanf(f_lut,"rear_pixel_width:%d\n",&rear_width);

    fscanf(f_lut,"left_pixel_height:%d   ",&left_height);
    fscanf(f_lut,"left_pixel_width:%d\n",&left_width);

    fscanf(f_lut,"right_pixel_height:%d   ",&right_height);
    fscanf(f_lut,"right_pixel_width:%d\n",&right_width);
    fclose(f_lut);

    int dst_width = front_width;
    int dst_height = front_height + rear_height + left_height;

    Mat front_birdview_lut(front_height,front_width,CV_8UC3);
    lutTable_generate_image(front_image, front_birdview_lut, image_view_birdview_e, view_front_e, fusion_min);

    Mat rear_birdview_lut(rear_height,rear_width,CV_8UC3);
    lutTable_generate_image(rear_image, rear_birdview_lut, image_view_birdview_e, view_rear_e, fusion_min);

    Mat left_birdview_lut(left_height,left_width,CV_8UC3);
    lutTable_generate_image(left_image, left_birdview_lut, image_view_birdview_e, view_left_e, fusion_min);

    Mat right_birdview_lut(right_height,right_width,CV_8UC3);
    lutTable_generate_image(right_image, right_birdview_lut, image_view_birdview_e, view_right_e, fusion_min);

    Mat dst_image = Mat::zeros(dst_height,dst_width, CV_8UC3);
    int row = 0;
    int col = 0;
    int dst_row = 0;
    int dst_col = 0;

    int dst_channel = front_image.channels();
    //front
    for(row = 0; row < front_height; row++)
    {
        for(col = 0; col < front_width; col++)
        {

            if (dst_channel == 3) {
                dst_image.at<Vec3b>(row, col)[0] = front_birdview_lut.at<Vec3b>(row, col)[0] ;
                dst_image.at<Vec3b>(row, col)[1] = front_birdview_lut.at<Vec3b>(row, col)[1];
                dst_image.at<Vec3b>(row, col)[2] = front_birdview_lut.at<Vec3b>(row, col)[2];
            }
            else
            {
                dst_image.at<uchar>(row, col) = front_birdview_lut.at<uchar>(row, col);

            }
        }

    }
    //left
    for(row = 0; row < left_height; row++)
    {
        dst_row = row + front_height;
        for(col = 0; col < left_width; col++)
        {
            if (dst_channel == 3) {
                dst_image.at<Vec3b>(dst_row, col)[0] = left_birdview_lut.at<Vec3b>(row, col)[0] ;
                dst_image.at<Vec3b>(dst_row, col)[1] = left_birdview_lut.at<Vec3b>(row, col)[1];
                dst_image.at<Vec3b>(dst_row, col)[2] = left_birdview_lut.at<Vec3b>(row, col)[2];
            }
            else
            {
                dst_image.at<uchar>(dst_row, col) = left_birdview_lut.at<uchar>(row, col);

            }
        }
    }
    //right
    for(row = 0; row < right_height; row++)
    {
        dst_row = row + front_height;
        for(col = 0; col < right_width; col++)
        {
            dst_col = col + front_width - right_width -1;

            if (dst_channel == 3) {
                dst_image.at<Vec3b>(dst_row, dst_col)[0] = right_birdview_lut.at<Vec3b>(row, col)[0] ;
                dst_image.at<Vec3b>(dst_row, dst_col)[1] = right_birdview_lut.at<Vec3b>(row, col)[1];
                dst_image.at<Vec3b>(dst_row, dst_col)[2] = right_birdview_lut.at<Vec3b>(row, col)[2];
            }
            else
            {
                dst_image.at<uchar>(dst_row, dst_col) = right_birdview_lut.at<uchar>(row, col);

            }
        }
    }
    //rear
    for(row = 0; row < rear_height; row++)
    {
        dst_row = row + front_height + right_height;
        for(col = 0; col < rear_width; col++)
        {
            if (dst_channel == 3) {
                dst_image.at<Vec3b>(dst_row, col)[0] = rear_birdview_lut.at<Vec3b>(row, col)[0] ;
                dst_image.at<Vec3b>(dst_row, col)[1] = rear_birdview_lut.at<Vec3b>(row, col)[1];
                dst_image.at<Vec3b>(dst_row, col)[2] = rear_birdview_lut.at<Vec3b>(row, col)[2];
            }
            else
            {
                dst_image.at<uchar>(dst_row, col) = rear_birdview_lut.at<uchar>(row, col);

            }
        }

    }
    return dst_image;
}


/*
 * author: liuli
 * date:
 * name: generate_gain
 * function: 根据四张原始鱼眼图，求解增益
 * parameters:
 *          IN： Mat front_raw_image 前原始图
 *          IN： Mat rear_raw_image 后原始图
 *          IN： Mat left_raw_image 左原始图
 *          IN： Mat right_raw_image 右原始图
 */
int generate_gain( float varN,  float varG, IN Mat front_raw_image, IN Mat rear_raw_image, IN Mat left_raw_image, IN Mat right_raw_image)
{
    int rtn = 0;

//    float varN = 20, varG = 0.05;

    int countFusion[8] = { 0, 0, 0, 0, 0, 0, 0, 0 }; // 各融合区域像素数统计

    int sumYuv[3][8] =
            {
                    { 0, 0, 0, 0, 0, 0, 0, 0 },
                    { 0, 0, 0, 0, 0, 0, 0, 0 },
                    { 0, 0, 0, 0, 0, 0, 0, 0 }
            }; // 各融合区域亮度和

    int aveYuv[3][8] =
            {
                    { 0, 0, 0, 0, 0, 0, 0, 0 },
                    { 0, 0, 0, 0, 0, 0, 0, 0 },
                    { 0, 0, 0, 0, 0, 0, 0, 0 }
            }; // 各融合区域亮度平均值

    int BASE_ADJUSATMENT = 128;
    FILE* fp_lut_birdview_front;
    FILE* fp_lut_birdview_rear;
    FILE* fp_lut_birdview_left;
    FILE* fp_lut_birdview_right;
   // double t = (double)cvGetTickCount();


    int k = 150;

    //0: front_left
    for (int row = k; row < front_height; row += 2)
    {
        int lut_row_offset = row * front_width * 6;
        for(int col = row; col < left_width; col += 2)
        {
            int lut_col_offset = col * 6;

            int src_row = *(file_data_front + lut_row_offset + lut_col_offset);
            int src_col = *(file_data_front + lut_row_offset + lut_col_offset + 1);


            int i = src_row;
            int j = src_col;
            float bgr[3] = {0};
            float yuv[3] = {0};
            bgr[0] = front_raw_image.ptr<Vec3b>(i)[j][0];
            bgr[1] = front_raw_image.ptr<Vec3b>(i)[j][1];
            bgr[2] = front_raw_image.ptr<Vec3b>(i)[j][2];
            bgrToYuv(bgr, yuv);
            sumYuv[0][0] += yuv[0];
            sumYuv[1][0] += yuv[1];
            sumYuv[2][0] += yuv[2];
            countFusion[0]++;
        }
    }
    //1: front_right
    for(int row = k; row < front_height; row += 2)
    {
        int lut_row_offset = row * front_width * 6;
        for(int col = front_width - row; col < front_width - k; col += 2)
        {
            int lut_col_offset = col * 6;
            int src_row = *(file_data_front + lut_row_offset + lut_col_offset);
            int src_col = *(file_data_front + lut_row_offset + lut_col_offset + 1);

            int i = src_row;
            int j = src_col;
            float bgr[3] = {0};
            float yuv[3] = {0};
            bgr[0] = front_raw_image.ptr<Vec3b>(i)[j][0];
            bgr[1] = front_raw_image.ptr<Vec3b>(i)[j][1];
            bgr[2] = front_raw_image.ptr<Vec3b>(i)[j][2];
            bgrToYuv(bgr,yuv);
            sumYuv[0][1] += yuv[0];
            sumYuv[1][1] += yuv[1];
            sumYuv[2][1] += yuv[2];
            countFusion[1]++;
        }
    }

    //2: right_front
    for(int row = k; row < front_height; row += 2)
    {
        int lut_row_offset = row * right_width * 6;
        for(int col = (right_width - row -1); col <right_width - k; col += 2)
        {
            int lut_col_offset = col * 6;

            int src_row = *(file_data_right + lut_row_offset + lut_col_offset);
            int src_col = *(file_data_right + lut_row_offset + lut_col_offset + 1);

            int i = src_row;
            int j = src_col;
            float bgr[3] = {0};
            float yuv[3] = {0};
            bgr[0] = right_raw_image.ptr<Vec3b>(i)[j][0];
            bgr[1] = right_raw_image.ptr<Vec3b>(i)[j][1];
            bgr[2] = right_raw_image.ptr<Vec3b>(i)[j][2];
            bgrToYuv(bgr,yuv);
            sumYuv[0][2] += yuv[0];
            sumYuv[1][2] += yuv[1];
            sumYuv[2][2] += yuv[2];
            countFusion[2]++;
        }
    }

    //3: right_rear
    for(int row = right_height - rear_height; row < right_height - k; row += 2)
    {
        int lut_row_offset = row * right_width * 6;
        for(int col = row - (right_height - rear_height); col <right_width - k; col += 2)
        {
            int lut_col_offset = col * 6;
            int src_row = *(file_data_right + lut_row_offset + lut_col_offset);
            int src_col = *(file_data_right + lut_row_offset + lut_col_offset + 1);

            int i = src_row;
            int j = src_col;
            float bgr[3] = {0};
            float yuv[3] = {0};
            bgr[0] = right_raw_image.ptr<Vec3b>(i)[j][0];
            bgr[1] = right_raw_image.ptr<Vec3b>(i)[j][1];
            bgr[2] = right_raw_image.ptr<Vec3b>(i)[j][2];
            bgrToYuv(bgr,yuv);
            sumYuv[0][3] += yuv[0];
            sumYuv[1][3] += yuv[1];
            sumYuv[2][3] += yuv[2];
            countFusion[3]++;
        }
    }
    //4: rear_right
    for(int row = 0; row < rear_height - k; row += 2)
    {
        int lut_row_offset = row * rear_width * 6;
        for(int col = rear_width - right_width; col < rear_width - right_width + row; col += 2)
        {
            int lut_col_offset = col * 6;
            int src_row = *(file_data_rear + lut_row_offset + lut_col_offset);
            int src_col = *(file_data_rear + lut_row_offset + lut_col_offset + 1);

            int i = src_row;
            int j = src_col;
            float bgr[3] = {0};
            float yuv[3] = {0};
            bgr[0] = rear_raw_image.ptr<Vec3b>(i)[j][0];
            bgr[1] = rear_raw_image.ptr<Vec3b>(i)[j][1];
            bgr[2] = rear_raw_image.ptr<Vec3b>(i)[j][2];
            bgrToYuv(bgr,yuv);
            sumYuv[0][4] += yuv[0];
            sumYuv[1][4] += yuv[1];
            sumYuv[2][4] += yuv[2];
            countFusion[4]++;
        }
    }
    //5: rear_left
    for(int row = 0; row < rear_height - k; row += 2)
    {
        int lut_row_offset = row * rear_width * 6;
        for(int col = left_width - row - 1; col < left_width; col += 2)
        {
            int lut_col_offset = col * 6;

            int src_row = *(file_data_rear + lut_row_offset + lut_col_offset);
            int src_col = *(file_data_rear + lut_row_offset + lut_col_offset + 1);

            int i = src_row;
            int j = src_col;
            float bgr[3] = {0};
            float yuv[3] = {0};
            bgr[0] = rear_raw_image.ptr<Vec3b>(i)[j][0];
            bgr[1] = rear_raw_image.ptr<Vec3b>(i)[j][1];
            bgr[2] = rear_raw_image.ptr<Vec3b>(i)[j][2];
            bgrToYuv(bgr,yuv);
            sumYuv[0][5] += yuv[0];
            sumYuv[1][5] += yuv[1];
            sumYuv[2][5] += yuv[2];
            countFusion[5]++;
        }
    }
    //6: left_rear
    for(int row = left_height - rear_height; row < left_height - k; row += 2)
    {
        int lut_row_offset = row * left_width * 6;
        for(int col = k; col < left_width - (row- (left_height - rear_height)); col += 2)
        {
            int lut_col_offset = col * 6;
            int src_row = *(file_data_left + lut_row_offset + lut_col_offset);
            int src_col = *(file_data_left + lut_row_offset + lut_col_offset + 1);

            int i = src_row;
            int j = src_col;
            float bgr[3] = {0};
            float yuv[3] = {0};
            bgr[0] = left_raw_image.ptr<Vec3b>(i)[j][0];
            bgr[1] = left_raw_image.ptr<Vec3b>(i)[j][1];
            bgr[2] = left_raw_image.ptr<Vec3b>(i)[j][2];
            bgrToYuv(bgr,yuv);
            sumYuv[0][6] += yuv[0];
            sumYuv[1][6] += yuv[1];
            sumYuv[2][6] += yuv[2];
            countFusion[6]++;
        }
    }
    //7: left_front
    for(int row = k; row < front_height; row += 2)
    {
        int lut_row_offset = row * left_width * 6;
        for(int col = k; col < row; col += 2)
        {
            int lut_col_offset = col * 6;

            int src_row = *(file_data_left + lut_row_offset + lut_col_offset);
            int src_col = *(file_data_left + lut_row_offset + lut_col_offset + 1);

            int i = src_row;
            int j = src_col;
            float bgr[3] = {0};
            float yuv[3] = {0};
            bgr[0] = left_raw_image.ptr<Vec3b>(i)[j][0];
            bgr[1] = left_raw_image.ptr<Vec3b>(i)[j][1];
            bgr[2] = left_raw_image.ptr<Vec3b>(i)[j][2];
            bgrToYuv(bgr,yuv);
            sumYuv[0][7] += yuv[0];
            sumYuv[1][7] += yuv[1];
            sumYuv[2][7] += yuv[2];
            countFusion[7]++;
        }
    }
    fclose(fp_lut_birdview_front);
    fclose(fp_lut_birdview_rear);
    fclose(fp_lut_birdview_left);
    fclose(fp_lut_birdview_right);

    free(file_data_front);
    file_data_front = NULL;
    free(file_data_right);
    file_data_right = NULL;
    free(file_data_rear);
    file_data_rear = NULL;
    free(file_data_left);
    file_data_left = NULL;

    for (int channel_id = 0; channel_id < 3; channel_id++)
    {
        for (int region_id = 0; region_id < 8; region_id++) // 8 fusion region totally
        {
            aveYuv[channel_id][region_id] = (float)sumYuv[channel_id][region_id] / countFusion[region_id];
        }
    }

    CvMat* matrixY = cvCreateMat(4, 4, CV_32F);
    CvMat* vectorY = cvCreateMat(4, 1, CV_32F);
    CvMat* gainY = cvCreateMat(4, 1, CV_32F);

    CvMat* matrixU = cvCreateMat(4, 4, CV_32F);
    CvMat* vectorU = cvCreateMat(4, 1, CV_32F);
    CvMat* gainU = cvCreateMat(4, 1, CV_32F);

    CvMat* matrixV = cvCreateMat(4, 4, CV_32F);
    CvMat* vectorV = cvCreateMat(4, 1, CV_32F);
    CvMat* gainV = cvCreateMat(4, 1, CV_32F);

    cvZero(matrixY);
    cvZero(matrixU);
    cvZero(matrixV);

    int CAMERA_NUM = 4;
    for (int i = 0; i < CAMERA_NUM; i++)
    {
        //y
        cvmSet(matrixY, i, i, ((float)aveYuv[0][i * 2] * aveYuv[0][i * 2] + aveYuv[0][i * 2 + 1] * aveYuv[0][i * 2 + 1]) / varN + 1 / (float)varG);
        cvmSet(matrixY, i, (i + 1) % CAMERA_NUM, -(float)aveYuv[0][i * 2 + 1] * aveYuv[0][(i * 2 + 2) % 8] / varN);
        cvmSet(matrixY, i, (i + 3) % CAMERA_NUM, -(float)aveYuv[0][(i * 2) % 8] * aveYuv[0][(i * 2 + 7) % 8] / varN);
        cvmSet(vectorY, i, 0, 1 / (float)varG);

        //u
        cvmSet(matrixU, i, i, ((float)aveYuv[1][i * 2] * aveYuv[1][i * 2] + aveYuv[1][i * 2 + 1] * aveYuv[1][i * 2 + 1]) / varN + 1 / (float)varG);
        cvmSet(matrixU, i, (i + 1) % CAMERA_NUM, -(float)aveYuv[1][i * 2 + 1] * aveYuv[1][(i * 2 + 2) % 8] / varN);
        cvmSet(matrixU, i, (i + 3) % CAMERA_NUM, -(float)aveYuv[1][(i * 2) % 8] * aveYuv[1][(i * 2 + 7) % 8] / varN);
        cvmSet(vectorU, i, 0, 1 / (float)varG);

        //v
        cvmSet(matrixV, i, i, ((float)aveYuv[2][i * 2] * aveYuv[2][i * 2] + aveYuv[2][i * 2 + 1] * aveYuv[2][i * 2 + 1]) / varN + 1 / (float)varG);
        cvmSet(matrixV, i, (i + 1) % CAMERA_NUM, -(float)aveYuv[2][i * 2 + 1] * aveYuv[2][(i * 2 + 2) % 8] / varN);
        cvmSet(matrixV, i, (i + 3) % CAMERA_NUM, -(float)aveYuv[2][(i * 2) % 8] * aveYuv[2][(i * 2 + 7) % 8] / varN);
        cvmSet(vectorV, i, 0, 1 / (float)varG);
    }

    cvSolve(matrixY, vectorY, gainY, CV_SVD);
    cvSolve(matrixU, vectorU, gainU, CV_SVD);
    cvSolve(matrixV, vectorV, gainV, CV_SVD);

    currentGains[0] = (int)(BASE_ADJUSATMENT * (cvmGet(gainY, 0, 0) - 1.0) + 0.5);
    currentGains[1] = (int)(BASE_ADJUSATMENT * (cvmGet(gainY, 2, 0) - 1.0) + 0.5);
    currentGains[2] = (int)(BASE_ADJUSATMENT * (cvmGet(gainY, 3, 0) - 1.0) + 0.5);
    currentGains[3] = (int)(BASE_ADJUSATMENT * (cvmGet(gainY, 1, 0) - 1.0) + 0.5);

    currentGains[4] = (int)(BASE_ADJUSATMENT * (cvmGet(gainU, 0, 0) - 1.0) + 0.5);
    currentGains[5] = (int)(BASE_ADJUSATMENT * (cvmGet(gainU, 2, 0) - 1.0) + 0.5);
    currentGains[6] = (int)(BASE_ADJUSATMENT * (cvmGet(gainU, 3, 0) - 1.0) + 0.5);
    currentGains[7] = (int)(BASE_ADJUSATMENT * (cvmGet(gainU, 1, 0) - 1.0) + 0.5);

    currentGains[8] = (int)(BASE_ADJUSATMENT * (cvmGet(gainV, 0, 0) - 1.0) + 0.5);
    currentGains[9] = (int)(BASE_ADJUSATMENT * (cvmGet(gainV, 2, 0) - 1.0) + 0.5);
    currentGains[10] = (int)(BASE_ADJUSATMENT * (cvmGet(gainV, 3, 0) - 1.0) + 0.5);
    currentGains[11] = (int)(BASE_ADJUSATMENT * (cvmGet(gainV, 1, 0) - 1.0) + 0.5);

    cvReleaseMat(&matrixY);
    cvReleaseMat(&vectorY);
    cvReleaseMat(&gainY);
    cvReleaseMat(&matrixU);
    cvReleaseMat(&vectorU);
    cvReleaseMat(&gainU);
    cvReleaseMat(&matrixV);
    cvReleaseMat(&vectorV);
    cvReleaseMat(&gainV);

    return rtn;
}

//liuli
void bgrToYuv(float bgr[3], float yuv[3])
{
    int Y, U, V, B, G, R;

    B = bgr[0];
    G = bgr[1];
    R = bgr[2];

    Y =  0.299 * R + 0.587 * G + 0.114 * B;
    U = -0.147 * R - 0.289 * G + 0.436 * B + 128;
    V =  0.615 * R - 0.515 * G - 0.100 * B + 128;

    Y = Y < 0 ? 0 : (Y > 255 ? 255 : Y);
    U = U < 0 ? 0 : (U > 255 ? 255 : U);
    V = V < 0 ? 0 : (V > 255 ? 255 : V);

    yuv[0] = (unsigned char)Y;
    yuv[1] = (unsigned char)U;
    yuv[2] = (unsigned char)V;
}

void yuvToBgr(float yuv[3], float bgr[3])
{

    int Y, U, V, B, G, R;

    Y = yuv[0];
    U = yuv[1];
    V = yuv[2];

    R = Y + 1.14 * (V - 128);
    G = Y - 0.39 * (U - 128) - 0.58 * (V - 128);
    B = Y + 2.03 * (U - 128);

    if(B<0.0)B=0;
    if(B>255.0)B=255;
    if(G<0.0)G=0;
    if(G>255.0)G=255;
    if(R<0.0)R=0;
    if(R>255.0)R=255;

    bgr[0] = B;
    bgr[1] = G;
    bgr[2] = R;
}

/*
 * author: cheng
 * date:
 * name:birdviewpoint2rawpixel
 * function: 已知某一拼接俯视图上的点坐标，求出此点对应的原图图上的坐标
 * parametes:
 *          IN： float stitchpoint[2] 俯视图下像素坐标，坐标,[0] x坐标; [1] y坐标
 *          IN： VIEW_E view_index 相机枚举，前、后、左、右
 *          INOUT
 *          OUT float rawpixel[2] 原图像素坐标 [0]: 行值 ; [1] 列值
 *
 *return;
 * */
int birdviewpoint2rawpixel(IN float stitchpoint[2], IN VIEW_E view_index, INOUT float rawpixel[2])
{
    int rtn = 0;

    double world_position[3] = {0};
    double trans[3][4] = {0};
    load_cam_extrinstic(trans, view_index);
    double x = 0;
    double y = 0;
    double z = 0;

    int world_height = front_world_view + rear_world_view + car_world_height;
    double dy =  world_height / double(left_pixel_height);
    double dx = dy;

    int stitch_point_x = stitchpoint[0];
    int stitch_point_y = stitchpoint[1];


    if(view_index == view_front_e)
    {
        stitch_point_x = stitchpoint[0];
        stitch_point_y = stitchpoint[1];
    }
    else if(view_index == view_rear_e)
    {
        stitch_point_x = stitchpoint[0];
        stitch_point_y = stitchpoint[1] + stitch_height - rear_height;
    }
    else if(view_index == view_left_e)
    {
        stitch_point_x = stitchpoint[0];
        stitch_point_y = stitchpoint[1];
    }
    else if(view_index == view_right_e)
    {
        stitch_point_x = stitchpoint[0] + stitch_width - right_width;
        stitch_point_y = stitchpoint[1];
    }

    x = (stitch_point_x - stitch_width/2)*dx;
    y = (stitch_height/2-stitch_point_y)*dy + (car_axle_coord - car_world_height/2);
    z = 0;

    world_position[0] = x;
    world_position[1] = y;
    world_position[2] = z;

    world2pixel(world_position, trans, rawpixel, view_index);
    return  rtn;
}
/*
 * author: cheng
 * date:
 * name:projectivepixel2projectivepixel
 * function: 像素点从一透视图转换到另一透视图的像素点
 * parametes:
 *          IN：VIEW_E view_index 相机枚举,F/B/L/R哪个相机
 *          IN：float projective_1_pixel[2] 透视图1上的像素点坐标, [0] v; [1] u
 *          IN：float projective_1_matirx[3][3] 透视图1的内参矩阵
 *          IN: float projective2_fov 透视图2展开FOV
 *          IN: float projective2_size[2] 透视图2展开像素大小 [0] height; [1] width,注意xml文件中的offset必须为透视图2的offset
 *          OUT float projective_2_pixel[2] 转换到透视图2下的像素点坐标 [0]: v [1]u
 *
 *return 是否能在透视图2上找到对应的像素点;
 * */
int projectivepixel2projectivepixel(IN VIEW_E view_index,IN float projective_1_pixel[2], IN float projective_1_matirx[3][3], IN float projective2_fov[2], IN float projective2_size[2], OUT float projective_2_pixel[2])
{
    int rtn = 0;
    float cam_x = 0.0;
    float cam_y = 0.0;
    float cam_z = 0.0;
    float col = projective_1_pixel[1];
    float row = projective_1_pixel[0];
    float fx1 = projective_1_matirx[0][0];
    float cx1 = projective_1_matirx[0][2];
    float fy1 = projective_1_matirx[1][1];
    float cy1 = projective_1_matirx[1][2];

    cam_model_s cam_model;
    load_cam_intrinstic(&cam_model, view_index);

    float fx1i = 1/fx1;
    float fy1i = 1/fy1;
    float cx1i = -cx1/fx1;
    float cy1i = -cy1/fy1;

    float cam_z_init = 100.0;

    cam_x = (col * fx1i + cx1i) * cam_z_init;
    cam_y = (row * fy1i + cy1i) * cam_z_init;
    cam_z =  cam_z_init;

    float cam_world_point[3] = {0};
    cam_world_point[1] = cam_x;
    cam_world_point[0] = cam_y;
    cam_world_point[2] = cam_z;  //至此得到相机坐标系下的空间物理点坐标。

    //鱼眼相机投影模型，空间点投影到鱼眼图上的像素坐标。
    float pixel[2] = {0};
    cam2pixel(cam_model, cam_world_point, pixel);

//    std::cout << "raw pixel" << pixel[0] << " , " << pixel[1] << std::endl;

    rawpixel2projective(pixel, view_index, projective2_fov, projective2_size, projective_2_pixel);

    if(projective_2_pixel[0] < 0 || projective_2_pixel[0] > projective2_size[0] || projective_2_pixel[1] < 0 || projective_2_pixel[1] > projective2_size[1] )
    {
        rtn = -1;
    }

    return rtn;
}


/*
 * author: cheng
 * date:
 * name:rawpixel2projective
 * function: 已知某一原图上的点坐标，求出此点对应的透视图上的坐标
 * parametes:
 *          IN： float raw_pixel[2] 原图上的，像素点坐标, [0] v; [1] u
 *          IN： VIEW_E view_index 相机枚举，前、后、左、右
 *          IN float projective_fov[2] 透视展开fov
 *          IN float projective_fov[2] 透视展开height width
 *          INOUT
 *          OUT float projective_pixel[2] 相机坐标系下的坐标 [0]: 行值  [1]列值
 *
 *return;
 * */
int rawpixel2projective(IN float raw_pixel[2], IN VIEW_E view_index, IN float projective_fov[2],IN float projective_size[2], OUT float projective_pixel[2])
{
    int rtn = 0;
    cam_model_s cam_model = {0};
    float cam_world_point[3];
    float cam_z_init = 100;
    float fov_h = projective_fov[0];
    float fov_v = projective_fov[1];
    float dst_height = projective_size[0];
    float dst_width = projective_size[1];

    float tan_h =tan(fov_h * PI/180/2);
    float tan_v =tan(fov_v * PI/180/2);

    float  pixel_hight_dis = (cam_z_init * tan_v) / (dst_height / 2); //垂直方向上，平均每个像素占的物理距离
    float  pixel_width_dis = (cam_z_init * tan_h) / (dst_width / 2);  //水平方向上，平均每个像素占的物理距离

    load_cam_intrinstic(&cam_model, view_index);

    pixel2cam(cam_model, cam_world_point, raw_pixel);

    float ratio = cam_z_init / cam_world_point[2];

    float cam_world_x = ratio * cam_world_point[0];
    float cam_world_y = ratio * cam_world_point[1];
    float cam_world_z = ratio * cam_world_point[2];

    if(view_index == view_front_e)
    {
        matrix_rowoffset =  front_matrix_rowoffset;
        matrix_coloffset =  front_matrix_coloffset;
    }
    else if (view_index == view_rear_e)
    {
        matrix_rowoffset =  rear_matrix_rowoffset;
        matrix_coloffset =  rear_matrix_coloffset;
    }
    else if (view_index == view_left_e)
    {
        matrix_rowoffset =  left_matrix_rowoffset;
        matrix_coloffset =  left_matrix_coloffset;
    }
    else if (view_index == view_right_e)
    {
        matrix_rowoffset =  right_matrix_rowoffset;
        matrix_coloffset =  right_matrix_coloffset;
    }

    projective_pixel[1] = cam_world_x/pixel_width_dis + dst_width/2 + matrix_coloffset; //列  u
    projective_pixel[0] = cam_world_y/pixel_hight_dis + dst_height/2 + matrix_rowoffset; //行  v

    return rtn;
}

int pix2cam_model(IN VIEW_E view_index,INOUT float cam_world_point[3], IN float raw_pixel[2])
{
    cam_model_s cam_model = {0};
    load_cam_intrinstic(&cam_model, view_index);

    pixel2cam(cam_model, cam_world_point, raw_pixel);
}


/*
 * author: cheng
 * date:
 * name:worldview2undistortpixel
 * function: 已知车辆世界坐标系下前、后、左、右视图下的四个的物理点，求出此视图下对应的透视图上的四个像素坐标,并将生成的四个透视图上的像素坐标写文件
 * parametes:
 *          IN： double world_position[3] 车辆坐标系下的物理坐标[x,y,z]
 *          IN： VIEW_E view_index 相机枚举，前、后、左、右
 *          IN float projective_fov[2] 透视展开fov
 *          IN float projective_fov[2] 透视展开height width
 *          INOUT
 *          OUT float projective_pixel[2] 相机坐标系下的坐标 [0]: 行值  [1]列值
 *
 *return;
 * */
int worldview2undistortpixel(IN VIEW_E view_index,  IN float projective_fov[2],IN float projective_size[2])
{
    FILE* f_lut;
    int pix_x;
    int pix_y;
    float projective_pixel[2];

    if (view_index == view_front_e)
    {
        string file_name = run_dir + "front_projective_point.txt";
        f_lut = fopen(file_name.c_str(),"w+");
      //  f_lut = fopen("./lut/front_projective_point.txt","w+");
    }
    else if (view_index == view_rear_e)
    {
        string file_name = run_dir + "rear_projective_point.txt";
        f_lut = fopen(file_name.c_str(),"w+");
      //  f_lut = fopen("./lut/rear_projective_point.txt","w+");
    }
    else if (view_index == view_left_e) {
        string file_name = run_dir + "left_projective_point.txt";
        f_lut = fopen(file_name.c_str(),"w+");
     //  f_lut = fopen("./lut/left_projective_point.txt", "w+");
    }
    else if (view_index == view_right_e)
    {
        string file_name = run_dir + "right_projective_point.txt";
        f_lut = fopen(file_name.c_str(),"w+");
      //  f_lut = fopen("./lut/right_projective_point.txt", "w+");
    }

    worldpoint2undistortpixel(view_index, projectiveview_point_lefttop_e, projective_fov,projective_size, projective_pixel);
    pix_x = projective_pixel[1];
    pix_y = projective_pixel[0];

    fprintf(f_lut,"lefttop_x:%-6d", pix_x);
    fprintf(f_lut,"lefttop_y:%-6d\n", pix_y);

    worldpoint2undistortpixel(view_index, projectiveview_point_leftbut_e, projective_fov,projective_size, projective_pixel);
    pix_x = projective_pixel[1];
    pix_y = projective_pixel[0];
    fprintf(f_lut,"leftbut_x:%-6d", pix_x);
    fprintf(f_lut,"leftbut_y:%-6d\n", pix_y);

    worldpoint2undistortpixel(view_index, projectiveview_point_righttop_e, projective_fov,projective_size, projective_pixel);
    pix_x = projective_pixel[1];
    pix_y = projective_pixel[0];
    fprintf(f_lut,"righttop_x:%-6d", pix_x);
    fprintf(f_lut,"righttop_y:%-6d\n", pix_y);

    worldpoint2undistortpixel(view_index, projectiveview_point_rightbut_e, projective_fov,projective_size, projective_pixel);
    pix_x = projective_pixel[1];
    pix_y = projective_pixel[0];
    fprintf(f_lut,"rightbut_x:%-6d",pix_x);
    fprintf(f_lut,"rightbut_y:%-6d\n", pix_y);


    fclose(f_lut);
}

/*
 * author: cheng
 * date:
 * name:world2undistortpixel
 * function: 已知某一车辆世界坐标系下的物理点，求出此点对应的透视图上的像素坐标
 * parametes:
 *          IN： VIEW_E view_index 相机枚举，前、后、左、右
 *          IN PROJECTIVE_POINT_ENUM projective_point 四个点的枚举
 *          IN float projective_fov[2] 透视展开fov
 *          IN float projective_fov[2] 透视展开height width
 *          INOUT
 *          OUT float projective_pixel[2] 相机坐标系下的坐标 [0]: 行值  [1]列值
 *
 *return;
 * */
int worldpoint2undistortpixel(IN VIEW_E view_index, IN PROJECTIVE_POINT_ENUM projective_point, IN float projective_fov[2],IN float projective_size[2], OUT float projective_pixel[2] )
{
    int rtn = 0;
    double trans[3][4];
    float rawpixel[2];
    double world_position[3];

    if (view_index == view_front_e)
    {
        if (projective_point == projectiveview_point_lefttop_e)
        {
            world_position[0] = front_projective_lefttop_x;
            world_position[1] = car_axle_coord + front_projective_lefttop_y;
        }
        else if(projective_point == projectiveview_point_leftbut_e)
        {
            world_position[0] = front_projective_leftbut_x;
            world_position[1] = car_axle_coord + front_projective_leftbut_y;
        }
        else if(projective_point == projectiveview_point_righttop_e)
        {
            world_position[0] = front_projective_righttop_x;
            world_position[1] = car_axle_coord + front_projective_righttop_y;
        }
        else if(projective_point == projectiveview_point_rightbut_e)
        {
            world_position[0] = front_projective_rightbut_x;
            world_position[1] = car_axle_coord + front_projective_rightbut_y;
        }
    }
    else if (view_index == view_rear_e)
    {
        if (projective_point == projectiveview_point_lefttop_e)
        {
            world_position[0] = -rear_projective_lefttop_x;
          //  world_position[1] = car_axle_coord -car_world_height - rear_projective_lefttop_y;
            world_position[1] = -745 - rear_projective_lefttop_y;
        }
        else if(projective_point == projectiveview_point_leftbut_e)
        {
            world_position[0] = -rear_projective_leftbut_x;
//            world_position[1] = car_axle_coord -car_world_height - rear_projective_leftbut_y;
            world_position[1] =-745 - rear_projective_leftbut_y;
        }
        else if(projective_point == projectiveview_point_righttop_e)
        {
            world_position[0] = -rear_projective_righttop_x;
         //   world_position[1] = car_axle_coord -car_world_height - rear_projective_righttop_y;
            world_position[1] = -745 - rear_projective_righttop_y;
        }
        else if(projective_point == projectiveview_point_rightbut_e)
        {
            world_position[0] = -rear_projective_rightbut_x;
//            world_position[1] = car_axle_coord -car_world_height - rear_projective_rightbut_y;
            world_position[1] =-745 - rear_projective_rightbut_y;
        }
    }
    else if (view_index == view_left_e)
    {
        if (projective_point == projectiveview_point_lefttop_e)
        {
            world_position[0] = -car_world_width / 2 - left_projective_lefttop_x;
            world_position[1] = left_cam_height_y + left_projective_lefttop_y;
        }
        else if(projective_point == projectiveview_point_leftbut_e)
        {
            world_position[0] = -car_world_width / 2 - left_projective_leftbut_x;
            world_position[1] = left_cam_height_y + left_projective_leftbut_y;
        }
        else if(projective_point == projectiveview_point_righttop_e)
        {
            world_position[0] = -car_world_width / 2 - left_projective_righttop_x;
            world_position[1] = left_cam_height_y + left_projective_righttop_y;
        }
        else if(projective_point == projectiveview_point_rightbut_e)
        {
            world_position[0] = -car_world_width / 2 - left_projective_rightbut_x;
            world_position[1] = left_cam_height_y + left_projective_rightbut_y;
        }
    }
    else if (view_index == view_right_e)
    {
        if (projective_point == projectiveview_point_lefttop_e)
        {
            world_position[0] = car_world_width / 2 + right_projective_lefttop_x;
            world_position[1] = right_cam_height_y + right_projective_lefttop_y;
        }
        else if(projective_point == projectiveview_point_leftbut_e)
        {
            world_position[0] = car_world_width / 2 + right_projective_leftbut_x;
            world_position[1] = right_cam_height_y + right_projective_leftbut_y;
        }
        else if(projective_point == projectiveview_point_righttop_e)
        {
            world_position[0] = car_world_width / 2 + right_projective_righttop_x;
            world_position[1] = right_cam_height_y + right_projective_righttop_y;
        }
        else if(projective_point == projectiveview_point_rightbut_e)
        {
            world_position[0] = car_world_width / 2 + right_projective_rightbut_x;
            world_position[1] = right_cam_height_y + right_projective_rightbut_y;
        }
    }

    world_position[3] = 0;

    load_cam_extrinstic(trans, view_index);

    //物理坐标转位原图像素坐标
    world2pixel(world_position, trans, rawpixel, view_index);

    //原图像素坐标对应 的透视图像素坐标
    rawpixel2projective(rawpixel, view_index, projective_fov, projective_size, projective_pixel);

    return rtn;
}

/*
 * author: cheng
 * date:
 * name:birdviewpix2worldpoint
 * function: 已知某一俯视图上的点坐标，求出此点对应的物理坐标
 * parametes:
 *          IN： float stitchpoint[2] 俯视图下像素坐标，坐标,[0] x坐标; [1] y坐标
 *          IN： VIEW_E view_index 相机枚举，前、后、左、右
 *          INOUT
 *          OUT float rawpixel[3] 物理坐标
 *
 *return;
 * */

int birdviewpix2worldpoint(IN float stitchpoint[2], IN VIEW_E view_index, INOUT float worldpoint[3])
{
    int rtn = 0;

    int world_height = front_world_view + rear_world_view + car_world_height;
    double dy =  world_height / double(left_pixel_height);
    double dx = dy;

    int stitch_width   = front_width;
    int stitch_height  = left_height;
    int stitch_point_x = stitchpoint[0];
    int stitch_point_y = stitchpoint[1];

    if(view_index == view_front_e)
    {
        stitch_point_x = stitchpoint[0];
        stitch_point_y = stitchpoint[1];
    }
    else if(view_index == view_rear_e)
    {
        stitch_point_x = stitchpoint[0];
        stitch_point_y = stitchpoint[1] + stitch_height - rear_height;
    }
    else if(view_index == view_left_e)
    {
        stitch_point_x = stitchpoint[0];
        stitch_point_y = stitchpoint[1];
    }
    else if(view_index == view_right_e)
    {
        stitch_point_x = stitchpoint[0] + stitch_width - right_width;
        stitch_point_y = stitchpoint[1];
    }

    worldpoint[0]  = (stitch_point_x - stitch_width/2)*dx;
    worldpoint[1]  = (stitch_height/2-stitch_point_y)*dy + (car_axle_coord - car_world_height/2);
    worldpoint[2]  = 0;

    return rtn;
}


/*
 * author: liuli
 * date:
 * name:birdviewpix2projectpoint
 * function: 已知某一拼接俯视图上的点坐标，求出此点对应的物理坐标
 * parametes:
 *          IN： float stitchpoint[2] 俯视图下像素坐标，坐标,[0] x坐标; [1] y坐标
 *          IN： VIEW_E view_index 相机枚举，前、后、左、右
 *          INOUT：float projective_point[2] 透视图的坐标
 *
 *return;
 * */

int birdviewpix2projectpoint(IN float stitchpoint[2], INOUT float projective_point[2],  IN VIEW_E view_index, IN float projective_fov[2], IN float projective_size[2])
{
    int rtn = 0;
    float rawpixel[2];     //rawpixel[0]; u   //rawpixel[1];v

    birdviewpoint2rawpixel(stitchpoint, view_index, rawpixel);

    rawpixel2projective(rawpixel, view_index, projective_fov, projective_size, projective_point);

    return rtn;
}



/*
 * author: liuli
 * date:
 * name:birdview2birdview
 * function: 给定原俯视图某一点的像素坐标(u,v)，输出对应拉伸后的俯视图像素坐标(u',v')。
 * parametes:
 *          IN： float stitchpoint[2] 原俯视图下像素坐标，坐标,[0] u坐标; [1] v坐标
 *          INOUT： float modifiedpoint[2] 拉伸后平行四边形像素坐标 [0] u‘坐标; [1] v’坐标
 *          IN: float stretch_coef, IN float yCoef 逆向转换系数
 * */
void birdview2birdview(IN float stitchpoint[2], INOUT float modifiedpoint[2], IN float stretch_coef, IN float yCoef)
{
    float world_position[3];
    VIEW_E view_index;

    int world_height = front_world_view + rear_world_view + car_world_height;
    int world_width = left_world_view + right_world_view + car_world_width;
    int front_pixel_height = round(front_world_view * left_pixel_height/world_height);
    int front_pixel_width = round(world_width * left_pixel_height/world_height);
    int left_pixel_width = round(left_world_view * left_pixel_height/world_height);//保证长宽比

    int rear_pixel_height = round(rear_world_view * left_pixel_height/world_height);
    int rear_pixel_width = front_pixel_width;

    int right_pixel_height = left_pixel_height;
    int right_pixel_width = round(right_world_view * left_pixel_height/world_height);
    double dy = world_height / double(left_pixel_height);
    double dx = dy;

    int col = stitchpoint[0];
    int row = stitchpoint[1];

    if(row < front_pixel_height)
    {
        view_index = view_front_e;
    }
    else if(row > (left_pixel_height - rear_pixel_height))
    {
        view_index = view_rear_e;
    }

    if( (front_pixel_height < row &&  row < (left_pixel_height - rear_pixel_height)))
    {
        if(col < front_pixel_width/2)
        {
            view_index = view_left_e;
        }
        else if(col >= front_pixel_width/2)
        {
            view_index = view_right_e;
        }
    }


    if(view_index == view_front_e)
    {
        birdviewpix2worldpoint(stitchpoint, view_index, world_position);
        world_position[1] =  world_position[1]/yCoef;
        float modifiedpointx = world_position[0]/dx + front_pixel_width/2;
        float stitch_point_y = left_pixel_height/2 - (world_position[1] - (car_axle_coord - car_world_height/2) - stretch_coef * (world_position[0]))/dy;
        modifiedpoint[0] = modifiedpointx;
        modifiedpoint[1] = stitch_point_y;
    }
    else if(view_index == view_rear_e)
    {
        stitchpoint[1] = stitchpoint[1] - (left_pixel_height - rear_pixel_height);
        birdviewpix2worldpoint(stitchpoint, view_index, world_position);
        world_position[1] = world_position[1]/yCoef;

        float modifiedpointx = world_position[0]/dx + rear_pixel_width / 2;
        float stitch_point_y = left_pixel_height/2 - (world_position[1] - (car_axle_coord - car_world_height/2) - stretch_coef * (world_position[0]))/dy;
        modifiedpoint[0] = modifiedpointx;
        modifiedpoint[1] = stitch_point_y;

    }
    else if(view_index == view_left_e)
    {
        birdviewpix2worldpoint(stitchpoint, view_index, world_position);
        world_position[1] = world_position[1]/yCoef;

        float modifiedpointx = world_position[0]/dx + front_pixel_width/2;
        float stitch_point_y = left_pixel_height/2 - (world_position[1] - (car_axle_coord - car_world_height/2) - stretch_coef * (world_position[0]))/dy;
        modifiedpoint[0] = modifiedpointx;
        modifiedpoint[1] = stitch_point_y;
    }
    else if(view_index == view_right_e)
    {
        stitchpoint[0] = stitchpoint[0] - (front_pixel_width - right_pixel_width);
        birdviewpix2worldpoint(stitchpoint, view_index, world_position);
        world_position[1] = world_position[1]/yCoef;

        float modifiedpointx = world_position[0]/dx + front_pixel_width/2;
        float stitch_point_y = left_pixel_height/2 - (world_position[1] - (car_axle_coord - car_world_height/2) - stretch_coef * (world_position[0]))/dy;
        modifiedpoint[0] = modifiedpointx;
        modifiedpoint[1] = stitch_point_y;
    }
}


/*
 * author:
 * date:
 * name: get_stretch_coef
 * function: 给定目标拉伸角度alpha和车位与水平方向的夹角theta,算出拉伸系数stretch_coef
 * parametes:
 *          IN： float alpha 给定目标拉伸角度; float theta 车位与水平方向的夹角
 *          INOUT： float stretch_coef 输出拉伸系数
 *          std::cout << tan(45*(2*3.14)/360) << endl;   //角度制算斜率
 *          std::cout << atan(1) * 360/(2*3.14) << endl;  //斜率算角度
 */
float get_stretch_coef(IN float alpha, IN float theta, IN float yCoef)
//float get_stretch_coef(IN float cord[6], IN float alpha, IN float yCoef)
{
    float stretch_coef = 0.5;
#if 0
    float x1 = cord[0];
    float y1 = cord[1];
    float x2 = cord[2];
    float y2 = cord[3];
    float x3 = cord[4];
    float y3 = cord[5];
    if((x1 == x2) || (x2 == x3) || (y2 == y3) || (y2 == y1))
    {
        float theta = 90;
        cout << "theta: " << theta << endl;
        stretch_coef = tan((90- alpha)* (3.14)/180);
        return  stretch_coef;
    }
    float k1 = (y2 - y3)/(x3 - x2);
    float k2 = (y1 - y2)/(x2 - x1);
    float theta = atan(k1) * 360/(2*3.14);
    cout << "theta: " << theta << endl;
#else
    float k1 = tan(theta*(2*3.14)/360);
    float k2 = tan((theta+90)*(2*3.14)/360);
#endif
    if((-1 < theta && theta < 15) || (70 < theta && theta < 91))
    {
        stretch_coef = tan((90- alpha)* (3.14)/180);
        return  stretch_coef;
    }
    // a^2 + b(k1+k2)*a + (b^2*k1k2 + 1) - b(k2-k1)/tan(alpha) = 0;   //a = stretch_coef; b = stretch_coef * y_Coef;
    // a^2*(1 + yCoef * (k1 + k2) + yCoef^2 * k1*k2) + a * (yCoef*(k2-k1) / tan(alpha)) + 1 = 0;
    // a * x^2 + b * x +c = 0;    x:stretch_coef
    float d =  tan(alpha*(2*3.14)/360);
    float a = (1 + yCoef * (k1 + k2) + yCoef * yCoef * k1*k2);
    float b = (yCoef*(k2-k1) / d);
    float c = 1;

    float delt = b * b - 4 * a * c;
    cout << "delt: " << delt << endl;
    if(delt > 0 || delt == 0)
    {
        stretch_coef = (-b  + sqrt(delt)) / (2*a);
        return stretch_coef;
    }
    else
    {
        return stretch_coef;
    }
}


/*
 * author:
 * date:
 * name: get_stretchParms
 * function: 给定车位与水平方向的夹角theta,选择合适的lut表并返回所对应的拉伸系数stretch_coef,yCoef
 */
int get_stretchParms(string file_name, float theta, float &stretch_coef, float &yCoef)
{
    fs.open(file_name, FileStorage::READ);
    if (!fs.isOpened())
    {
        cout<<"open xml file "<< file_name << "failed" << endl;
        return -1;
    }

    if(-1 < theta && theta <= 20)
    {
        fs["run_directory_0_20"]>>run_dir;
        fs["stretch_coef_0_20"]>>stretch_coef;
        fs["yCoef_0_20"]>>yCoef;
    }
    else if(20 < theta && theta <= 40)
    {
        fs["run_directory_20_40"]>>run_dir;
        fs["stretch_coef_20_40"]>>stretch_coef;
        fs["yCoef_20_40"]>>yCoef;
    }
    else if(40 < theta && theta <= 60)
    {
        fs["run_directory_40_60"]>>run_dir;
        fs["stretch_coef_40_60"]>>stretch_coef;
        fs["yCoef_40_60"]>>yCoef;
    }
    else if(60 < theta && theta <= 90)
    {
        fs["run_directory_60_90"]>>run_dir;
        fs["stretch_coef_60_90"]>>stretch_coef;
        fs["yCoef_60_90"]>>yCoef;
    }
    else
    {
        fs["run_directory"]>>run_dir;
    }

    string birdview_file_name = run_dir + "birdview_size.txt";

    FILE* f_lut = fopen(birdview_file_name.c_str(),"r");

    fscanf(f_lut,"front_pixel_height:%d   ",&front_height);
    fscanf(f_lut,"front_pixel_width:%d\n",&front_width);

    fscanf(f_lut,"rear_pixel_height:%d   ",&rear_height);
    fscanf(f_lut,"rear_pixel_width:%d\n",&rear_width);

    fscanf(f_lut,"left_pixel_height:%d   ",&left_height);
    fscanf(f_lut,"left_pixel_width:%d\n",&left_width);

    fscanf(f_lut,"right_pixel_height:%d   ",&right_height);
    fscanf(f_lut,"right_pixel_width:%d\n",&right_width);

    stitch_width = front_width;
    stitch_height = left_height;
    fclose(f_lut);

    //把表都读出来
    string lut_file_name_front = run_dir + "birdview_front.bin";
    fp_lut_front = fopen(lut_file_name_front.c_str(),"rb");

    string fusion_file_name_front = run_dir + "fusion_front.bin";
    fp_fusion_front = fopen(fusion_file_name_front.c_str(),"rb");


    string lut_file_name_rear = run_dir + "birdview_rear.bin";
    fp_lut_rear = fopen(lut_file_name_rear.c_str(),"rb");

    string fusion_file_name_rear = run_dir + "fusion_rear.bin";
    fp_fusion_rear = fopen(fusion_file_name_rear.c_str(),"rb");


    string lut_file_name_left = run_dir + "birdview_left.bin";
    fp_lut_left = fopen(lut_file_name_left.c_str(),"rb");

    string fusion_file_name_left = run_dir + "fusion_left.bin";
    fp_fusion_left = fopen(fusion_file_name_left.c_str(),"rb");


    string lut_file_name_right = run_dir + "birdview_right.bin";
    fp_lut_right = fopen(lut_file_name_right.c_str(),"rb");

    string fusion_file_name_right = run_dir + "fusion_right.bin";
    fp_fusion_right = fopen(fusion_file_name_right.c_str(),"rb");


    file_data_front = (uint32_t*)malloc(front_height*front_width*(sizeof(uint32_t)*2 + 4*sizeof(float)));
    fread(file_data_front,front_height*front_width*(sizeof(uint32_t)*2 + 4*sizeof(float)),1,fp_lut_front);

    file_data_rear = (uint32_t*)malloc(rear_height*rear_width*(sizeof(uint32_t)*2 + 4*sizeof(float)));
    fread(file_data_rear,rear_height*rear_width*(sizeof(uint32_t)*2 + 4*sizeof(float)),1,fp_lut_rear);

    file_data_left = (uint32_t*)malloc(left_height*left_width*(sizeof(uint32_t)*2 + 4*sizeof(float)));
    fread(file_data_left,left_height*left_width*(sizeof(uint32_t)*2 + 4*sizeof(float)),1,fp_lut_left);

    file_data_right = (uint32_t*)malloc(right_height*right_width*(sizeof(uint32_t)*2 + 4*sizeof(float)));
    fread(file_data_right,right_height*right_width*(sizeof(uint32_t)*2 + 4*sizeof(float)),1,fp_lut_right);


    file_fusion_front = (float*)malloc(front_height * front_width * sizeof(float));
    fread(file_fusion_front, front_height * front_width * sizeof(float), 1, fp_fusion_front);

    file_fusion_rear = (float*)malloc(rear_height * rear_width * sizeof(float));
    fread(file_fusion_rear, rear_height * rear_width * sizeof(float), 1, fp_fusion_rear);

    file_fusion_left = (float*)malloc(left_height * left_width * sizeof(float));
    fread(file_fusion_left, left_height * left_width * sizeof(float), 1, fp_fusion_left);

    file_fusion_right = (float*)malloc(right_height * right_width * sizeof(float));
    fread(file_fusion_right, right_height * right_width * sizeof(float), 1, fp_fusion_right);

    return 0;
}

/*
 * author:
 * date: [1/10]
 * name: stitch_thetaLot
 * function: 给定车位与水平方向的夹角theta,得到拉伸后的斜车位
 * parametes:
 *          IN： Mat raw_image,原始四路鱼眼图
 *          IN： float theta，原车位与水平方向的夹角
 *          INOUT： Mat stitch_fusion, 变换后的斜车位拼接图
 *          INOUT： float stretch_coef, 返回的车位拉伸系数
 *          INOUT： float yCoef, 返回的车位展宽系数
 *
 */
void stitch_thetaLot(IN Mat raw_image, INOUT Mat &stitch_fusion, IN float theta, INOUT float &stretch_coef, INOUT float &yCoef){
    get_stretchParms("./XML/parameter_GL8_08_23.xml", theta, stretch_coef, yCoef);

    Mat front_imagex = raw_image(Rect(0,0,1280,720));
    Mat back_imagex = raw_image(Rect(0,720,1280,720));//读取图像
    Mat left_imagex = raw_image(Rect(1280,0,1280,720));//读取图像
    Mat right_imagex = raw_image(Rect(1280,720,1280,720));//读取图像
    Mat front_image = front_imagex.clone();
    Mat back_image = back_imagex.clone();
    Mat left_image = left_imagex.clone();
    Mat right_image = right_imagex.clone();
    stitch_fusion = lutTable_generate_stitchFusion(front_image, back_image, left_image, right_image, true, true,
                                                   true, true);
}

void backUp()
{
    if(0)  //back
    {

#ifdef PANORAMA_UYVY0
        int stitch_height = 512;
    int stitch_width = 512;
    int Height = stitch_height;
    int Width = stitch_width;
    int SINGLE_VIEW_WIDTH = front_image.cols;
    int SINGLE_VIEW_HEIGHT = front_image.rows;
    uchar* result_image_uyvy = (uchar*)malloc(sizeof(uchar)* Height * Width * 2);
    uchar* front_image_uyvy = (uchar*)malloc(sizeof(uchar)* SINGLE_VIEW_WIDTH * SINGLE_VIEW_HEIGHT * 2);
    uchar* back_image_uyvy  = (uchar*)malloc(sizeof(uchar)* SINGLE_VIEW_WIDTH * SINGLE_VIEW_HEIGHT * 2);
    uchar* left_image_uyvy  = (uchar*)malloc(sizeof(uchar)* SINGLE_VIEW_WIDTH * SINGLE_VIEW_HEIGHT * 2);
    uchar* right_image_uyvy = (uchar*)malloc(sizeof(uchar)* SINGLE_VIEW_WIDTH * SINGLE_VIEW_HEIGHT * 2);
    FILE*  pf_src = fopen("./IMAGE/MG_12_06/front.yuv", "rb");
    fread(front_image_uyvy, sizeof(uchar), SINGLE_VIEW_WIDTH * SINGLE_VIEW_HEIGHT * 2, pf_src);
    fclose(pf_src);
    pf_src = fopen("./IMAGE/MG_12_06/back.yuv", "rb");
    fread(back_image_uyvy, sizeof(uchar), SINGLE_VIEW_WIDTH * SINGLE_VIEW_HEIGHT * 2, pf_src);
    fclose(pf_src);
    pf_src = fopen("./IMAGE/MG_12_06/left.yuv", "rb");
    fread(left_image_uyvy, sizeof(uchar), SINGLE_VIEW_WIDTH * SINGLE_VIEW_HEIGHT * 2, pf_src);
    fclose(pf_src);
    pf_src = fopen("./IMAGE/MG_12_06/right.yuv", "rb");
    fread(right_image_uyvy, sizeof(uchar), SINGLE_VIEW_WIDTH * SINGLE_VIEW_HEIGHT * 2, pf_src);
    fclose(pf_src);
    /**************yuv stitch_fusion ****************/
         char str[256];
    for(int index = 0; index < 10; index++) {
        double t = (double)cvGetTickCount();
    generate_stitchFusion_yuv(
            result_image_uyvy,
            front_image_uyvy,
            back_image_uyvy,
            left_image_uyvy,
            right_image_uyvy, true, true, true, true);

        t = (double) cvGetTickCount() - t;
        printf("run time = %gms\n", t / (cvGetTickFrequency() * 1000));
    }
    // save result
    sprintf(str, "result_image/all_bilinear_result_%dx%d.yuv", Width, Height);
    FILE* pf_dst = fopen(str, "wb");
    fwrite(result_image_uyvy, sizeof(uchar), Width *  Height * 2, pf_dst);
    fclose(pf_dst);

#endif

//********************************** BirdviewPoint 2 ProjectivePoint **************************************//
//    float stitchpoint[2];
//    float rawpixel[2];
//    float raw_pixel[2];
//    float projective_pixel[2];
//    float projective_point[2];
//    float projective_fov[2] = {120,110};
//    float projective_size[2] = {480, 640};
//
//    stitchpoint[0] = 39;
//    stitchpoint[1] = 333;
//
//    birdviewpoint2rawpixel(stitchpoint, view_right_e,rawpixel);
//
//    raw_pixel[0] = rawpixel[0];//u
//    raw_pixel[1] = rawpixel[1];//v
//
//    rawpixel2projective(raw_pixel, view_right_e, projective_fov, projective_size, projective_pixel);

//*******************************************birdviewpixel2worldpoint*************************************//
//    float stitchpoint[2] = {0};
//    float worldpoint[3] ={0};
//
//    float pixel_x[] = {77,79,79,78,79,80};
//    float pixel_y[] = {443, 391, 337, 285, 232, 180};
//    for(int i = 0; i < 6; i++)
//    {
//        stitchpoint[0] = pixel_x[i];
//        stitchpoint[1] = pixel_y[i];
//
//        birdviewpix2worldpoint(stitchpoint, view_right_e, worldpoint);
//        std::cout << "x=" << worldpoint[0] << " y=" <<  worldpoint[1] << endl;
//    }

//*******************************************birdview2birdview*************************************//
//    float markerPoint[24] = {176,312,217,369,95,453,55,399,
//            281,466,320,523,198,601,163,551,
//            323,527,363,583,236,660,205,606
//    };
//    float stitchpoint[2] = {0};
//    float modifiedpoint[2] = {0};
//
//    for(int i = 0; i < 12; ++i){
//        stitchpoint[0] = markerPoint[2*i];
//        stitchpoint[1] = markerPoint[2*i + 1];
//        birdview2birdview(stitchpoint, modifiedpoint, stretch_coef);
//        std::cout << (int)modifiedpoint[0] <<"," << (int)modifiedpoint[1] << ",";
//    }


//    //*************************************** generate projective .bin   *********************************//
//
//    Mat out_front(480, 640, CV_8UC3);
//    Mat out_back(480, 640, CV_8UC3);
//    Mat out_left(480, 640, CV_8UC3);
//    Mat out_right(480, 640, CV_8UC3);
//    float fov[2] = {120,90};
//    undistort_plane_image(front_image, out_front, fov, view_front_e);
//    imshow("out_front", out_front);
//
//    undistort_plane_image(back_image, out_back, fov, view_rear_e);
//    imshow("out_back", out_back);
//
//    undistort_plane_image(left_image, out_left, fov, view_left_e);
//    imshow("out_left",  out_left);
//
//    undistort_plane_image(right_image, out_right, fov, view_right_e);
//    imshow("out_right", out_right);

//  // RW test
//        get_parameter("./XML/parameter_SQRW.xml");
//        Mat front_imagex = imread("./IMAGE/RW_IMAGE/front.bmp");//读取图像
//        Mat back_imagex = imread("./IMAGE/RW_IMAGE/back.bmp");//读取图像
//        Mat left_imagex = imread("./IMAGE/RW_IMAGE/left.bmp");//读取图像
//        Mat right_imagex = imread("./IMAGE/RW_IMAGE/right.bmp");//读取图像

    }
}
