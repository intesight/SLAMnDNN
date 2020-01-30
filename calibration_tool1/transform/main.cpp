//
// Created by wangkun on 18-7-5.
//

#include <iostream>
#include "image_view_convert.h"

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include<opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#include "opencv2/video.hpp"

#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
using namespace std;
using namespace cv;

#define __DEBUG_MG__ 1
#define __projective_test__
//#define __test__
int test_featuredetect(Mat &img_1,Mat &img_2);
int main()
{
#if __DEBUG_MG__
//    cout << "MG" <<endl;
//    get_parameter("./XML/parameter_MG_2019_03_23.xml");
    get_parameter("./XML/parameter_SQRW_2019_01_23.xml");
//    cout<<"it is ok"<<endl;
//    get_parameter("./XML/parameter_MG_2019_01_21.xml");//parameter_MG_10_31
    Mat front_imagex = imread("./IMAGE/MG_2019_03_23/front.bmp");//读取图像
    Mat back_imagex  = imread("./IMAGE/MG_2019_03_23/back.bmp");//读取图像
    Mat left_imagex  = imread("./IMAGE/MG_2019_03_23/left.bmp");//读取图像
    Mat right_imagex = imread("./IMAGE/MG_2019_03_23/right.bmp");//读取图像

    string source ="/home/tangshuaishuai/Data_Video/20190411-141527-front.avi";           // the source file name
    VideoCapture inputVideo(source);              // Open input
    if (!inputVideo.isOpened())
    {
        cout  << "Could not open the input video: " << source << endl;
    }



//    Mat front_imagex = imread("./IMAGE/RW_IMAGE_2019_01_23/front.bmp");//读取图像
//    Mat back_imagex  = imread("./IMAGE/RW_IMAGE_2019_01_23/back.bmp");//读取图像
//    Mat left_imagex  = imread("./IMAGE/RW_IMAGE_2019_01_23/left.bmp");//读取图像
//    Mat right_imagex = imread("./IMAGE/RW_IMAGE_2019_01_23/right.bmp");//读取图像
    cout<<"it is ok"<<endl;
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

//    float alpha = 60;
//    float theta = 88;
//    float stretch_coef = get_stretch_coef(alpha, theta, yCoef);
//    float stretch_coef = get_stretch_coef(cord, alpha, yCoef);
//    std::cout << "stretch_coef: " << stretch_coef <<endl;
//    Mat front_bird_view = birdview_image_generate(front_image, view_front_e, stitch_select_b, stretch_coef, yCoef);
//    cout<<"front size : "<<front_bird_view.size()<<endl;
//    imshow("front", front_bird_view);
//    imwrite("front.png", front_bird_view);

//    Mat back_bird_view = birdview_image_generate(back_image, view_rear_e, stitch_select_b, stretch_coef, yCoef);
//    cout<<"back size : "<<back_bird_view.size()<<endl;
//    imshow("back", back_bird_view);
//    imwrite("back.png", back_bird_view);
//
//    Mat left_bird_view = birdview_image_generate(left_image, view_left_e, stitch_select_b, stretch_coef, yCoef);
//    cout<<"left size : "<<left_bird_view.size()<<endl;
//    imshow("left", left_bird_view);
//    imwrite("left.png", left_bird_view);
//
//    Mat right_bird_view = birdview_image_generate(right_image, view_right_e, stitch_select_b, stretch_coef, yCoef);
//    cout<<"right size : "<<right_bird_view.size()<<endl;
//    imshow("right", right_bird_view);
//    imwrite("birdview_right_0.bmp", right_bird_view);
#endif
#ifdef __projective_test__
    Mat out_front(480, 640, CV_8UC3);
    Mat out_back(480, 640, CV_8UC3);
    Mat out_left(480, 640, CV_8UC3);
    Mat out_right(480, 640, CV_8UC3);
    float fov[2] = {90,80};
    Rect rect_front(0,0,1280,720);
    Mat src_front;
    int count = 0;

    while(1){
        inputVideo>>src_front;
        count++;
        cout<<"count number = "<<count<<endl;
//        if(count == 30)
//            imwrite("source_video.bmp",src_front);
        if (src_front.empty())
        {
            cout<<"cap end!"<<endl;
            break;
        }
        imshow("src_image",src_front);
        waitKey(10);
    undistort_plane_image(src_front, out_front, fov, view_front_e);
    imshow("out_front", out_front);
//    undistort_cylinder_image(front_image, out_front, fov, view_front_e);
//    undistort_cylinder_image(right_image, out_right, fov, view_right_e);
//    imshow("front", front_image);
//    imshow("right_image", right_image);
//    imshow("cy_expand_front", out_front);
//    imshow("cy_expand_right", out_right);
//
//    undistort_plane_image(back_image, out_back, fov, view_rear_e);
//    imshow("out_back", out_back);
//
//    undistort_plane_image(left_image, out_left, fov, view_left_e);
//    imshow("out_left",  out_left);
//
//    undistort_plane_image(right_image, out_right, fov, view_right_e);
//    imshow("out_right", out_right);



//        front_bird_view = birdview_image_generate(src_front, view_front_e, stitch_select_b, stretch_coef, yCoef);
//        cout<<"front size : "<<front_bird_view.size()<<endl;
//        imshow("front", front_bird_view);


    }
#endif

#if 1
    Mat src;
   //********************************* stitch_fusion ********************************//
    Mat stitch_fusion;
    char str[256];
//    float theta = 0;
//    for(int index = 0; index < 1; index++) {
    int index = 0;
    if(1){
        inputVideo >> src;
        if (src.empty()){
            cout<<"end failed!"<<endl;
//            break;
        }
        Rect rect1(0, 0, 1280, 720);
        Rect rect2(0, 720, 1280, 720);
        Rect rect3(1280, 0, 1280, 720);
        Rect rect4(1280, 720, 1280, 720);
//        front_image = src(rect1);
//        back_image = src(rect2);
//        left_image = src(rect3);
//        right_image = src(rect4);
//        imshow("test",src);
//        waitKey(10);
        double t = (double)cvGetTickCount();

//        get_stretchParms("./XML/parameter_GL8_08_23.xml", theta, stretch_coef, yCoef);    //斜车位配合lutTable_generate_stitchFusion 函数使用
        stitch_fusion = lutTable_generate_stitchFusion(front_image, back_image, left_image, right_image, true, true,
                                                           true, true);
//
//        stitch_thetaLot(raw_image, stitch_fusion, theta, stretch_coef, yCoef);
//
        sprintf(str, "result_image/1226/98_xCoef%g_yCoef%g_%d.bmp", stretch_coef, yCoef, index);
//        if(stitch_fusion.empty())
//            cout<<"empty"<<endl;
//        imshow(str, stitch_fusion);
//        imwrite(str, stitch_fusion);
//        t = (double) cvGetTickCount() - t;
//        printf("run time = %gms\n", t / (cvGetTickFrequency() * 1000));
        waitKey(10);
    }
#endif
   /* /*//*****************************************birdview2birdview***********************************//*/
    float markerPoint[16] = {240,333,243,258,73,251,73,323,
            239,339,237,412,72,401,73,328};
    float stitchpoint[2] = {0};
    float modifiedpoint[2] = {0};
    get_stretchParms("./XML/parameter_GL8_08_23.xml", theta, stretch_coef, yCoef);
    for(int i = 0; i < 8; ++i){
        stitchpoint[0] = markerPoint[2*i];
        stitchpoint[1] = markerPoint[2*i + 1];
        birdview2birdview(stitchpoint, modifiedpoint, stretch_coef, yCoef);

        std::cout << (int)modifiedpoint[0] <<"," << (int)modifiedpoint[1] << ",";
    }*/


#ifdef __test__
    Mat pro_image(480, 640, CV_8UC3);
    lutTable_generate_image(front_image, pro_image, image_view_projective_e, view_front_e, fusion_min);
    imshow("pro_image", pro_image);
    imwrite("pro_image.png", pro_image);
#endif


#ifdef PANORAMA_UYVY
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
    sprintf(str, "all_bilinear_result_%dx%d.yuv", Width, Height);
    FILE* pf_dst = fopen(str, "wb");
    fwrite(result_image_uyvy, sizeof(uchar), Width *  Height * 2, pf_dst);
    fclose(pf_dst);

#endif
    waitKey();
    return 0;
}
int test_featuredetect(Mat &img_1,Mat &img_2)
{
    std::vector<KeyPoint> keypoints_1, keypoints_2;
    Mat descriptors_1, descriptors_2;
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );

    //-- 第一步:检测 Oriented FAST 角点位置
    detector->detect ( img_1,keypoints_1 );
    detector->detect ( img_2,keypoints_2 );

    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute ( img_1, keypoints_1, descriptors_1 );
    descriptor->compute ( img_2, keypoints_2, descriptors_2 );

    Mat outimg1;
    drawKeypoints( img_1, keypoints_1, outimg1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
    imshow("ORB features",outimg1);

    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    vector<DMatch> matches;
    //BFMatcher matcher ( NORM_HAMMING );
    matcher->match ( descriptors_1, descriptors_2, matches );

    //-- 第四步:匹配点对筛选
    double min_dist=10000, max_dist=0;

    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        double dist = matches[i].distance;
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
    }

    min_dist = min_element( matches.begin(), matches.end(), [](const DMatch& m1, const DMatch& m2) {return m1.distance<m2.distance;} )->distance;
    max_dist = max_element( matches.begin(), matches.end(), [](const DMatch& m1, const DMatch& m2) {return m1.distance<m2.distance;} )->distance;

    printf ( "-- Max dist : %f \n", max_dist );
    printf ( "-- Min dist : %f \n", min_dist );

    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    std::vector< DMatch > good_matches;
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        if ( matches[i].distance <= max ( 2*min_dist, 5.0 ) )
        {
            good_matches.push_back ( matches[i] );
        }
    }

    //-- 第五步:绘制匹配结果
    Mat img_match;
    Mat img_goodmatch;
    drawMatches ( img_1, keypoints_1, img_2, keypoints_2, matches, img_match );
    drawMatches ( img_1, keypoints_1, img_2, keypoints_2, good_matches, img_goodmatch );
    imshow ( "all matched points", img_match );
    imshow ( "optimized matched points", img_goodmatch );
    waitKey(0);

    return 0;

}
