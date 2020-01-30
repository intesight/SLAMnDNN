//
// Created by wangkun on 18-7-5.
//

#include <iostream>
#include "image_view_convert.h"

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
using namespace std;
using namespace cv;

#define __DEBUG_MG__ 1
#define __projective_test__ 1

int main()
{
#if __DEBUG_MG__
    cout << "MG" <<endl;

        get_parameter("./XML/parameter_SQRW_2019_01_23.xml");
        Mat front_imagex = imread("./IMAGE/RW_IMAGE_2019_01_23/front.bmp");//读取图像
        Mat back_imagex = imread("./IMAGE/RW_IMAGE_2019_01_23/back.bmp");//读取图像
        Mat left_imagex = imread("./IMAGE/RW_IMAGE_2019_01_23/left.bmp");//读取图像
        Mat right_imagex = imread("./IMAGE/RW_IMAGE_2019_01_23/right.bmp");//读取图像

        Mat front_image = front_imagex.clone();
        Mat back_image = back_imagex.clone();
        Mat left_image = left_imagex.clone();
        Mat right_image = right_imagex.clone();
#else
    cout << "GL8" <<endl;
    get_parameter("./XML/parameter_GL8_08_23.xml");
//    Mat raw_image = imread("./IMAGE/GL8_IMAGE_08_23/front.bmp");
//    Mat front_imagex = raw_image(Rect(0,0,1280,720));
//    Mat back_imagex = raw_image(Rect(0,720,1280,720));//读取图像
//    Mat left_imagex = raw_image(Rect(1280,0,1280,720));//读取图像
//    Mat right_imagex = raw_image(Rect(1280,720,1280,720));//读取图像
    Mat front_imagex = imread("./IMAGE/GL8_IMAGE_08_23/front.bmp");//读取图像
    Mat back_imagex  = imread("./IMAGE/GL8_IMAGE_08_23/back.bmp");//读取图像
    Mat left_imagex  = imread("./IMAGE/GL8_IMAGE_08_23/left.bmp");//读取图像
    Mat right_imagex = imread("./IMAGE/GL8_IMAGE_08_23/right.bmp");//读取图像
    Mat front_image = front_imagex.clone();
    Mat back_image = back_imagex.clone();
    Mat left_image = left_imagex.clone();
    Mat right_image = right_imagex.clone();
#endif

    float stretch_coef = 0;
    float yCoef = 1 ;
#if 1
    //***************************************generate lut table .bin*********************************//
    bool stitch_select_b = true;

//    //斜车位
//    float alpha = 60;
//    float theta = 88;
//    float stretch_coef = get_stretch_coef(alpha, theta, yCoef);
    std::cout << "stretch_coef: " << stretch_coef <<endl;
    Mat front_bird_view = birdview_image_generate(front_image, view_front_e, stitch_select_b, stretch_coef, yCoef);
    cout<<"front size : "<<front_bird_view.size()<<endl;
    imshow("front", front_bird_view);
//    imwrite("front.png", front_bird_view);

    Mat back_bird_view = birdview_image_generate(back_image, view_rear_e, stitch_select_b, stretch_coef, yCoef);
    cout<<"back size : "<<back_bird_view.size()<<endl;
    imshow("back", back_bird_view);
//    imwrite("back.png", back_bird_view);

    Mat left_bird_view = birdview_image_generate(left_image, view_left_e, stitch_select_b, stretch_coef, yCoef);
    cout<<"left size : "<<left_bird_view.size()<<endl;
    imshow("left", left_bird_view);
//    imwrite("left.png", left_bird_view);

    Mat right_bird_view = birdview_image_generate(right_image, view_right_e, stitch_select_b, stretch_coef, yCoef);
    cout<<"right size : "<<right_bird_view.size()<<endl;
    imshow("right", right_bird_view);
//    imwrite("birdview_right_0.bmp", right_bird_view);
#endif
#if __projective_test__
    Mat out_front(480, 640, CV_8UC3);
    Mat out_back(480, 640, CV_8UC3);
    Mat out_left(480, 640, CV_8UC3);
    Mat out_right(480, 640, CV_8UC3);
    float fov[2] = {120,90};
    undistort_plane_image(front_image, out_front, fov, view_front_e);
    imshow("out_front", out_front);

    undistort_plane_image(back_image, out_back, fov, view_rear_e);
    imshow("out_back", out_back);

    undistort_plane_image(left_image, out_left, fov, view_left_e);
    imshow("out_left",  out_left);

    undistort_plane_image(right_image, out_right, fov, view_right_e);
    imshow("out_right", out_right);
#endif

#if 1
   //********************************* stitch_fusion ********************************//
    Mat stitch_fusion;
    char str[256];
//    float theta = 0;
    for(int index = 0; index < 1; index++) {
        double t = (double)cvGetTickCount();

//        get_stretchParms("./XML/parameter_GL8_08_23.xml", theta, stretch_coef, yCoef);    //斜车位配合lutTable_generate_stitchFusion 函数使用
        stitch_fusion = lutTable_generate_stitchFusion(front_image, back_image, left_image, right_image, true, true,
                                                           true, true);

//        stitch_thetaLot(raw_image, stitch_fusion, theta, stretch_coef, yCoef);

        sprintf(str, "result_image/1226/98_xCoef%g_yCoef%g_%d.bmp", stretch_coef, yCoef, index);
        imshow("111", stitch_fusion);
//        imwrite(str, stitch_fusion);
        t = (double) cvGetTickCount() - t;
        printf("run time = %gms\n", t / (cvGetTickFrequency() * 1000));
    }
#endif



    waitKey();
    return 0;
}
