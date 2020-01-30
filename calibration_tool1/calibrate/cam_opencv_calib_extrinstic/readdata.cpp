#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <fstream>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

using namespace std;
using namespace cv;


#define  __DEBUG_MG__ 1

static double computeReprojectionErrors( const vector<vector<Point3f> >& objectPoints,
                                         const vector<vector<Point2f> >& imagePoints,
                                         const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                                         const Mat& cameraMatrix , const Mat& distCoeffs,
                                         vector<float>& perViewErrors, bool fisheye)
{
    vector<Point2f> imagePoints2;
    size_t totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for(size_t i = 0; i < objectPoints.size(); ++i )
    {
        if (fisheye)
        {
            fisheye::projectPoints(objectPoints[i], imagePoints2, rvecs[i], tvecs[i], cameraMatrix,
                                   distCoeffs);
        }
        else
        {
            projectPoints(objectPoints[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePoints2);
        }
        err = norm(imagePoints[i], imagePoints2, NORM_L2);

        size_t n = objectPoints[i].size();
        perViewErrors[i] = (float) std::sqrt(err*err/n);
        totalErr        += err*err;
        totalPoints     += n;
    }

    return std::sqrt(totalErr/totalPoints);
}

int main()
{
    //MG
#if __DEBUG_MG__

//    string world_coordinate_file_x_path = "./MG_pic/MG_08_21/src_data/world_x_left.txt";
//    string world_coordinate_file_y_path = "./MG_pic/MG_08_21/src_data/world_y_left.txt";
//    string pixel_coordinate_file_x_path = "./MG_pic/MG_12_06/left/left_x.txt";
//    string pixel_coordinate_file_y_path = "./MG_pic/MG_12_06/left/left_y.txt";
//
//    string camera_instrinstic_path = "./MG_pic/MG_09_18/Intersic_Parameters/ocam_intrinsic_MG_left.txt";
//    string distortiion_coefficients_path = "./MG_pic/MG_09_18/Intersic_Parameters/distortiion_coefficients_MG_left.txt";

    string world_coordinate_file_x_path = "/home/caobei/calibration_tool/calibrate/cam_opencv_calib_extrinstic/RW_20190123/world/world_x_right.txt";
    string world_coordinate_file_y_path = "/home/caobei/calibration_tool/calibrate/cam_opencv_calib_extrinstic/RW_20190123/world/world_y_right.txt";
    string pixel_coordinate_file_x_path = "/home/caobei/calibration_tool/calibrate/cam_opencv_calib_extrinstic/RW_20190123/right/right_x.txt";
    string pixel_coordinate_file_y_path = "/home/caobei/calibration_tool/calibrate/cam_opencv_calib_extrinstic/RW_20190123/right/right_y.txt";

    string camera_instrinstic_path = "/home/caobei/calibration_tool/calibrate/cam_opencv_calib_extrinstic/RW_20190123/Intersic_Parameter/ocam_intrinsic_right.txt";
    string distortiion_coefficients_path = "/home/caobei/calibration_tool/calibrate/cam_opencv_calib_extrinstic/RW_20190123/Intersic_Parameter/distortiion_coefficients_right.txt";


#else   //GL8

    string world_coordinate_file_x_path = "./GL8_6_26_installed/src_data/world_back_x.txt";
    string world_coordinate_file_y_path = "./GL8_6_26_installed/src_data/world_back_y.txt";
    string pixel_coordinate_file_x_path = "./GL8_6_26_installed/src_data/back_x.txt";
    string pixel_coordinate_file_y_path = "./GL8_6_26_installed/src_data/back_y.txt";

    string camera_instrinstic_path = "./GL8_6_26_installed/camera_matrix/camera_intrinsic_back.txt";
    string distortiion_coefficients_path = "./GL8_6_26_installed/camera_matrix/camera_distortion_coefficients_back.txt";

#endif


//    char output_file_trans[] = "./output/trans";
//    char output_file_K_optimized[] = "./output/optimized_K";
//    char output_file_D_optimized[] = "./output/optimized_D";
//    char output_file_totalAvgErr[] = "./output/totalAvgErr";
//    char output_file_result[] = "./output/result";

    char output_file_trans[] = "/home/caobei/calibration_tool/calibrate/cam_opencv_calib_extrinstic/RW_20190123/right/output/trans";
    char output_file_K_optimized[] = "/home/caobei/calibration_tool/calibrate/cam_opencv_calib_extrinstic/RW_20190123/right/output/optimized_K";
    char output_file_D_optimized[] = "/home/caobei/calibration_tool/calibrate/cam_opencv_calib_extrinstic/RW_20190123/right/output/optimized_D";
    char output_file_totalAvgErr[] = "/home/caobei/calibration_tool/calibrate/cam_opencv_calib_extrinstic/RW_20190123/right/output/totalAvgErr";
    char output_file_result[] = "/home/caobei/calibration_tool/calibrate/cam_opencv_calib_extrinstic/RW_20190123/right/output/result";

    vector<Point3f> object;
    vector<vector<Point3f>> objectv;
    vector<Point2f> imagepix;
    vector<vector<Point2f>> imagev;

    //数据文件相关输入流
    ifstream infilex;
    ifstream infiley;
    ifstream infile_imagex;
    ifstream infile_imagey;

    ifstream camera_matrix;
    ifstream distortiion_coefficients;

    //文件指针，用于写入输出数据
    FILE *fp_trans = fopen(output_file_trans,"w+");
    FILE *fp_trans_camera_K = fopen(output_file_K_optimized,"w+");
    FILE *fp_trans_camera_D = fopen(output_file_D_optimized,"w+");
    FILE *fp_trans_totalAvgErr = fopen(output_file_totalAvgErr,"w+");
    FILE *fp_trans_result = fopen(output_file_result,"w+");

    //打开相关数据文件
    infilex.open(world_coordinate_file_x_path,ios::in);
    infiley.open(world_coordinate_file_y_path,ios::in);
    infile_imagex.open(pixel_coordinate_file_x_path,ios::in);
    infile_imagey.open(pixel_coordinate_file_y_path,ios::in);
    camera_matrix.open(camera_instrinstic_path,ios::in);
    distortiion_coefficients.open(distortiion_coefficients_path,ios::in);


    if(!infilex)
    {
        cout<<"fail to open the file "<<world_coordinate_file_x_path<<endl;
        exit(1);
    }
    if(!infiley)
    {
        cout<<"fail to open the file "<<world_coordinate_file_y_path<<endl;
        exit(1);
    }
    if(!infile_imagex)
    {
        cout<<"fail to open the file "<<pixel_coordinate_file_x_path<<endl;
        exit(1);
    }
    if(!infile_imagey)
    {
        cout<<"fail to open the file "<<pixel_coordinate_file_y_path<<endl;
        exit(1);
    }
    if(!camera_matrix)
    {
        cout<<"fail to open the file "<<camera_instrinstic_path<<endl;
        exit(1);
    }
    if(!distortiion_coefficients)
    {
        cout<<"fail to open the file "<<distortiion_coefficients_path<<endl;
        exit(1);
    }


    //相机内参和畸变系数
    Mat K(3,3,CV_64FC1);
    Mat D(4,1,CV_64FC1);


//
//    Mat raw_image = imread("./GL8/image/front.bmp");
//    int raw_width = raw_image.cols;
//    int raw_height = raw_image.rows;
//    cout<<"raw_width : "<<raw_width<<endl;
//    cout<<"raw_height : "<<raw_height<<endl;
//

    int raw_width = 1280;
    int raw_height = 720;

    //读取相机内参数据写入K
    double RR;
    int rr_index = 0;
    cout<<"camera_matrix : "<<endl;
    while(camera_matrix>>RR)     //    　　1、文本文件的读写: 用插入器(<<)向文件输出;    用析取器(>>)从文件输入。 //从camera_matrix文本中读取一个double值。
    {
        K.at<double>(rr_index, 0) = RR;
        cout<<RR<< " ";

        camera_matrix >> RR;
        K.at<double>(rr_index, 1) = RR;
        cout<<RR<<" ";

        camera_matrix >> RR;
        K.at<double>(rr_index, 2) = RR;
        cout<<RR<<endl;

        rr_index++;
    }

    if(rr_index >3 )
    {
        cout<<"camera_matrix.txt is illegal"<<endl;
        exit(1);
    }

    //读取畸变系数，写入D
    double coeff;

    distortiion_coefficients >> coeff;
    D.at<double>(0, 0) = coeff;
    cout<<"coeff: "<<coeff<< " ";

    distortiion_coefficients >> coeff;
    D.at<double>(1, 0) = coeff;
    cout<<coeff<< " ";

    distortiion_coefficients >> coeff;
    D.at<double>(2, 0) = coeff;
    cout<<coeff<< " ";

    distortiion_coefficients >> coeff;
    D.at<double>(3, 0) = coeff;
    cout<<coeff<<endl;


    //读取位置坐标和像素坐标
    double datax;
    double datay;

    double data_imagex;
    double data_imagey;

    int i = 0;
    while(infilex>>datax)
    {
        infiley>>datay;
        infile_imagex>>data_imagex;
        infile_imagey>>data_imagey;

        object.push_back(Point3f(datax,datay,0));
        imagepix.push_back(Point2f(data_imagex,data_imagey));

        cout<<"object coordinate and imagepixel coordinate : "<<i<<": "<<object[i]<<imagepix[i]<<endl;

        i++;
    }

    objectv.push_back(object);
    imagev.push_back(imagepix);

    vector<Point3f> object2;
    vector<Point2f> imagepix2;

    infilex.close();
    infiley.close();
    infile_imagex.close();
    infile_imagey.close();

    Mat rvecsm;
    Mat tvecsm;


    int flags = 0;

    flags += cv::fisheye::CALIB_USE_INTRINSIC_GUESS; //若指定此标签，必须保证cameraMatrix包含有效的fx、fy、cx、cy的初始值。否则（cx，cy）最初被设置为图像中心。焦距以最小二乘计算
    flags += cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC; //每次优化迭代内参后重新计算外参
    //flags |= cv::fisheye::CALIB_CHECK_COND; //检查条件号的有效性
    flags += cv::fisheye::CALIB_FIX_SKEW; //保持fx与fy相同
    //flags |= cv::fisheye::CALIB_FIX_K1; //鱼眼畸变系数选择哪一个置为0并保持为0。即为某一系数固定为0
    //flags |= cv::fisheye::CALIB_FIX_K2;
    //flags |= cv::fisheye::CALIB_FIX_K3;
    //flags |= cv::fisheye::CALIB_FIX_K4;
    flags += cv::fisheye::CALIB_FIX_INTRINSIC; //固定内参

    cv::Mat K_before = K.clone();
    cv::Mat D_before = D.clone();

    cout << "K_before_optimize: " << K_before << endl;
    cout << "D_before_optimize: " << D_before << endl;

    double rms = 0;
    rms = cv::fisheye::calibrate(objectv, imagev, cv::Size(raw_width,raw_height), K, D, rvecsm, tvecsm, flags);

    cout << "K_after_optimize: " << K << endl;
    cout << "D_after_optimize: " << D << endl;

    cout<<"rvecsm : "<<rvecsm<<endl;
    cout<<"tvecsm : "<<tvecsm<<endl;

    Mat row1 = rvecsm.rowRange(0,1).clone();
    cout<<"row1 : "<<row1<<endl;

    Mat rvecsm2rmatrix;
    cv::Rodrigues(row1, rvecsm2rmatrix); //罗德李格斯公式将旋转向量转换成旋转矩阵

    cout<<"rvecsm2rmatrix : "<<rvecsm2rmatrix<<endl;

#if 1

    // 赋值。R直接赋值
    Mat srcR(3,3,CV_32F);

    srcR.at<float>(0,0) = float(rvecsm2rmatrix.at<double>(1,0));
    srcR.at<float>(0,1) = float(rvecsm2rmatrix.at<double>(1,1));
    srcR.at<float>(0,2) = float(rvecsm2rmatrix.at<double>(1,2));

    srcR.at<float>(1,0) = float(rvecsm2rmatrix.at<double>(0,0));
    srcR.at<float>(1,1) = float(rvecsm2rmatrix.at<double>(0,1));
    srcR.at<float>(1,2) = float(rvecsm2rmatrix.at<double>(0,2));

    srcR.at<float>(2,0) = float(rvecsm2rmatrix.at<double>(2,0));
    srcR.at<float>(2,1) = float(rvecsm2rmatrix.at<double>(2,1));
    srcR.at<float>(2,2) = float(rvecsm2rmatrix.at<double>(2,2));

    //T由行向量变为列向量
    Mat srcT(3,1,CV_32F);

    srcT.at<float>(0,0) = float(tvecsm.at<double>(0,1));
    srcT.at<float>(1,0) = float(tvecsm.at<double>(0,0));
    srcT.at<float>(2,0) = float(tvecsm.at<double>(0,2));

    std::cout<< "srcR:"<< srcR << endl;
    std::cout<< "srcT:"<< srcT << endl;

    Mat verification_result = -(srcR.inv())*srcT;
    cout << "verification_result:" << verification_result<< endl;

#endif

    //将结果rt写入文件,存为T矩阵，外参矩阵。
    //调整相机坐标系，外参第一行和第二行互换 [liuli 2019/01/11]
//    double R11 = rvecsm2rmatrix.at<double>(0,0);
//    double R12 = rvecsm2rmatrix.at<double>(0,1);
//    double R13 = rvecsm2rmatrix.at<double>(0,2);
//    double R21 = rvecsm2rmatrix.at<double>(1,0);
//    double R22 = rvecsm2rmatrix.at<double>(1,1);
//    double R23 = rvecsm2rmatrix.at<double>(1,2);

    double R11 = rvecsm2rmatrix.at<double>(1,0);
    double R12 = rvecsm2rmatrix.at<double>(1,1);
    double R13 = rvecsm2rmatrix.at<double>(1,2);
    double R21 = rvecsm2rmatrix.at<double>(0,0);
    double R22 = rvecsm2rmatrix.at<double>(0,1);
    double R23 = rvecsm2rmatrix.at<double>(0,2);
    double R31 = rvecsm2rmatrix.at<double>(2,0);
    double R32 = rvecsm2rmatrix.at<double>(2,1);
    double R33 = rvecsm2rmatrix.at<double>(2,2);

//    double T1 = tvecsm.at<double>(0,0);
//    double T2 = tvecsm.at<double>(0,1);

    double T1 = tvecsm.at<double>(0,1);
    double T2 = tvecsm.at<double>(0,0);
    double T3 = tvecsm.at<double>(0,2);

    fprintf(fp_trans,"%15.9f",R11);
    fprintf(fp_trans,"%15.9f",R12);
    fprintf(fp_trans,"%15.9f",R13);
    fprintf(fp_trans,"%15.9f\n",T1);

    fprintf(fp_trans,"%15.9f",R21);
    fprintf(fp_trans,"%15.9f",R22);
    fprintf(fp_trans,"%15.9f",R23);
    fprintf(fp_trans,"%15.9f\n",T2);

    fprintf(fp_trans,"%15.9f",R31);
    fprintf(fp_trans,"%15.9f",R32);
    fprintf(fp_trans,"%15.9f",R33);
    fprintf(fp_trans,"%15.9f\n",T3);

    //优化后的相机内参矩阵K存储
    double K11 = K.at<double>(0,0);
    double K12 = K.at<double>(0,1);
    double K13 = K.at<double>(0,2);
    double K21 = K.at<double>(1,0);
    double K22 = K.at<double>(1,1);
    double K23 = K.at<double>(1,2);
    double K31 = K.at<double>(2,0);
    double K32 = K.at<double>(2,1);
    double K33 = K.at<double>(2,2);

    fprintf(fp_trans_camera_K,"%15.9f",K11);
    fprintf(fp_trans_camera_K,"%15.9f",K12);
    fprintf(fp_trans_camera_K,"%15.9f\n",K13);
    fprintf(fp_trans_camera_K,"%15.9f",K21);
    fprintf(fp_trans_camera_K,"%15.9f",K22);
    fprintf(fp_trans_camera_K,"%15.9f\n",K23);
    fprintf(fp_trans_camera_K,"%15.9f",K31);
    fprintf(fp_trans_camera_K,"%15.9f",K32);
    fprintf(fp_trans_camera_K,"%15.9f",K33);

    //优化后的畸变系数存储
    double D1 = D.at<double>(0,0);
    double D2 = D.at<double>(0,1);
    double D3 = D.at<double>(0,2);
    double D4 = D.at<double>(0,3);

    fprintf(fp_trans_camera_D,"%15.9f\n",D1);
    fprintf(fp_trans_camera_D,"%15.9f\n",D2);
    fprintf(fp_trans_camera_D,"%15.9f\n",D3);
    fprintf(fp_trans_camera_D,"%15.9f\n",D4);

    //平均误差存储
    fprintf(fp_trans_totalAvgErr,"%15.9f\n",rms);

    //result存储
    double result00 = verification_result.at<float>(0,0);
    double result10 = verification_result.at<float>(1,0);
    double result20 = verification_result.at<float>(2,0);

    fprintf(fp_trans_result,"%15.9f\n",result00);
    fprintf(fp_trans_result,"%15.9f\n",result10);
    fprintf(fp_trans_result,"%15.9f\n",result20);

    double totalAvgErr = 0;
    vector<float> projectErrors;
    vector<Mat> rvecs;
    vector<Mat> tvecs;
    double r11 = rvecsm.at<double>(0,0);
    double r12 = rvecsm.at<double>(0,1);
    double r13 = rvecsm.at<double>(0,2);
    double r14 = rvecsm.at<double>(1,0);
    double r15 = rvecsm.at<double>(1,1);
    double r16 = rvecsm.at<double>(1,2);
    rvecs.reserve(rvecsm.rows);
    tvecs.reserve(tvecsm.rows);

    for(int i = 0; i < int(objectv.size()); i++){
        rvecs.push_back(rvecsm.row(i));
        tvecs.push_back(tvecsm.row(i));
    }

    totalAvgErr = computeReprojectionErrors(objectv, imagev, rvecs, tvecs, K, D, projectErrors, 1);

    cout<<"rms = "<<rms<<endl;
    cout<<"totalAvgErr = "<<totalAvgErr<<endl;

    vector<float>::iterator vector_it;

    for(vector_it = projectErrors.begin(); vector_it != projectErrors.end(); vector_it++)
    {
        cout<<"projectErrors = "<< *vector_it << endl;
    }

    fclose(fp_trans);
    fclose(fp_trans_camera_K);
    fclose(fp_trans_camera_D);
    fclose(fp_trans_totalAvgErr);

    return 0;
}
