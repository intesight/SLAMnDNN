#include <iostream>
#include <sstream>
#include <map>
#include <time.h>
#include <stdio.h>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

#ifndef _CRT_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS
#endif

using namespace cv;
using namespace std;

map<int, string> m;

static void help()
{
    cout <<  "This is a camera calibration sample." << endl
         <<  "Usage: calibration configurationFile"  << endl
         <<  "Near the sample file you'll find the configuration file, which has detailed help of "
             "how to edit it.  It may be any OpenCV supported file format XML/YAML." << endl;
}
class Settings
{
public:
    Settings() : goodInput(false) {}
    enum Pattern { NOT_EXISTING, CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };
    enum InputType { INVALID, CAMERA, VIDEO_FILE, IMAGE_LIST };

    void write(FileStorage& fs) const                        //Write serialization for this class
    {
        fs << "{"
                  << "BoardSize_Width"  << boardSize.width
                  << "BoardSize_Height" << boardSize.height
                  << "Square_Size"         << squareSize
                  << "Calibrate_Pattern" << patternToUse
                  << "Calibrate_NrOfFrameToUse" << nrFrames
                  << "Calibrate_FixAspectRatio" << aspectRatio
                  << "Calibrate_AssumeZeroTangentialDistortion" << calibZeroTangentDist
                  << "Calibrate_FixPrincipalPointAtTheCenter" << calibFixPrincipalPoint

                  << "Write_DetectedFeaturePoints" << writePoints
                  << "Write_extrinsicParameters"   << writeExtrinsics
                  << "Write_outputFileName"  << outputFileName

                  << "Show_UndistortedImage" << showUndistorsed

                  << "Input_FlipAroundHorizontalAxis" << flipVertical
                  << "Input_Delay" << delay
                  << "Input" << input
           << "}";
    }
    void read(const FileNode& node)                          //Read serialization for this class
    {
        node["BoardSize_Width" ] >> boardSize.width;
        node["BoardSize_Height"] >> boardSize.height;
        node["Calibrate_Pattern"] >> patternToUse;
        node["Square_Size"]  >> squareSize;
        node["Calibrate_NrOfFrameToUse"] >> nrFrames;
        node["Calibrate_FixAspectRatio"] >> aspectRatio;
        node["Write_DetectedFeaturePoints"] >> writePoints;
        node["Write_extrinsicParameters"] >> writeExtrinsics;
                                                                                      node["Write_outputFileName"] >> outputFileName;
        node["Calibrate_AssumeZeroTangentialDistortion"] >> calibZeroTangentDist;
        node["Calibrate_FixPrincipalPointAtTheCenter"] >> calibFixPrincipalPoint;
        node["Calibrate_UseFisheyeModel"] >> useFisheye;
        node["Input_FlipAroundHorizontalAxis"] >> flipVertical;
        node["Show_UndistortedImage"] >> showUndistorsed;
        node["Input"] >> input;
        node["Input_Delay"] >> delay;
        validate();

     //   useFisheye = true;
    }
    void validate()
    {
        goodInput = true;
        if (boardSize.width <= 0 || boardSize.height <= 0)
        {
            cerr << "Invalid Board size: " << boardSize.width << " " << boardSize.height << endl;
            goodInput = false;
        }
        if (squareSize <= 10e-6)
        {
            cerr << "Invalid square size " << squareSize << endl;
            goodInput = false;
        }
        if (nrFrames <= 0)
        {
            cerr << "Invalid number of frames " << nrFrames << endl;
            goodInput = false;
        }

        //若input为空。input即为in_VID5.xml中的Input标签。如果为空的话说明没有传入任何数据信息，所以输入类型为invalid
        if (input.empty())      // Check for valid input
                inputType = INVALID;
        else
        {
            //若input标签开头为0-9数字的话，输入类型为CAMERA，需要记录cameraID。
            if (input[0] >= '0' && input[0] <= '9')
            {
                stringstream ss(input);
                ss >> cameraID;
                inputType = CAMERA;
            }
            else
            {
                //如果解析图片list成功，则将输入类型设置为IMAGE_LIST
                if (readStringList(input, imageList))
                {
                    inputType = IMAGE_LIST;
                    //nrFrames为用于标定的图片张数。此为在in_VID5.xml中的设定，若少于输入的图片张数，则为设定值。大于提供的图片张数的话则为图片张数。
                    nrFrames = (nrFrames < (int)imageList.size()) ? nrFrames : (int)imageList.size();
                }
                    //其他情况为视频文件
                else
                    inputType = VIDEO_FILE;
            }
            //如果为camera模式，打开相机ID
            if (inputType == CAMERA)
                inputCapture.open(cameraID);
            //如果为视频文件，打开input输入
            if (inputType == VIDEO_FILE)
                inputCapture.open(input);
            if (inputType != IMAGE_LIST && !inputCapture.isOpened())
                    inputType = INVALID;
        }
        if (inputType == INVALID)
        {
            cerr << " Input does not exist: " << input;
            goodInput = false;
        }

        flag = 0;
     //   flag = CALIB_FIX_K5;
       // flag = CALIB_FIX_K4 | CALIB_FIX_K5;
        if(calibFixPrincipalPoint) flag |= CALIB_FIX_PRINCIPAL_POINT;
        if(calibZeroTangentDist)   flag |= CALIB_ZERO_TANGENT_DIST;
        if(aspectRatio)            flag |= CALIB_FIX_ASPECT_RATIO;

     //   flag |=   cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC ;

        if (useFisheye) {
            // the fisheye model has its own enum, so overwrite the flags
         //   flag = fisheye::CALIB_FIX_SKEW | fisheye::CALIB_RECOMPUTE_EXTRINSIC; /// |
                   // fisheye::CALIB_FIX_K1 |
              //     fisheye::CALIB_FIX_K2 | fisheye::CALIB_FIX_K3 | fisheye::CALIB_FIX_K4;
        }

        calibrationPattern = NOT_EXISTING;
        if (!patternToUse.compare("CHESSBOARD")) calibrationPattern = CHESSBOARD;
        if (!patternToUse.compare("CIRCLES_GRID")) calibrationPattern = CIRCLES_GRID;
        if (!patternToUse.compare("ASYMMETRIC_CIRCLES_GRID")) calibrationPattern = ASYMMETRIC_CIRCLES_GRID;
        if (calibrationPattern == NOT_EXISTING)
        {
            cerr << " Camera calibration mode does not exist: " << patternToUse << endl;
            goodInput = false;
        }
        atImageList = 0;

    }
    Mat nextImage()
    {
        Mat result;
        if( inputCapture.isOpened() )
        {
            Mat view0;
            inputCapture >> view0;
            view0.copyTo(result);
        }
        else if( atImageList < imageList.size() )
        {
            cout<<"atImageList = "<<atImageList<<endl;
            string name_pic = imageList[atImageList++];
            result = imread(name_pic, IMREAD_COLOR);
        }

        return result;
    }

    /**
     * 解析出文件中的各个图像路径名称，并存入vector中。
     * @param filename 图像路径文件名，这里为VID5.xml。
     * @param l 输出承接vector，解析出的每张图片路径名称存入其中。
     * @return 返回值为是否解析并存储成功。
     */
    static bool readStringList( const string& filename, vector<string>& l )
    {
        //首先清空输出vector
        l.clear();
        //将数据文件读入到fs中
        FileStorage fs(filename, FileStorage::READ);
        //打开失败return false
        if( !fs.isOpened() )
            return false;
        //返回映射顶层的第一个元素,这里为<images>标签
        FileNode n = fs.getFirstTopLevelNode();
        //此处输出为images
        //cout<<"n.name = "<<n.name()<<endl;
        //此处输出为5,enum定义中5即为FileNode::SEQ
        //cout<<"n.type = "<<n.type()<<endl;

        //确保为sequence,保证后续用迭代器进行遍历
        if( n.type() != FileNode::SEQ )
            return false;

        //迭代器遍历进行push_back
        FileNodeIterator it = n.begin(), it_end = n.end();
        for( int i = 0; it != it_end; ++it )
        {
            cout<<"the "<< i <<" picture named : "<<(string)*it<<endl;
            m[i] = (string)*it;
            l.push_back((string)*it);
            i++;
        }
        return true;
    }
public:
    Size boardSize;              // The size of the board -> Number of items by width and height
    Pattern calibrationPattern;  // One of the Chessboard, circles, or asymmetric circle pattern
    float squareSize;            // The size of a square in your defined unit (point, millimeter,etc).
    int nrFrames;                // The number of frames to use from the input for calibration
    float aspectRatio;           // The aspect ratio
    int delay;                   // In case of a video input
    bool writePoints;            // Write detected feature points
    bool writeExtrinsics;        // Write extrinsic parameters
    bool calibZeroTangentDist;   // Assume zero tangential distortion
    bool calibFixPrincipalPoint; // Fix the principal point at the center
    bool flipVertical;           // Flip the captured images around the horizontal axis
    string outputFileName;       // The name of the file where to write
    bool showUndistorsed;        // Show undistorted images after calibration
    string input;                // The input ->
    bool useFisheye;             // use fisheye camera model for calibration

    int cameraID;                // 相机ID
    vector<string> imageList;    // 图片路径名称vector
    size_t atImageList;          // 图片路径名称索引
    VideoCapture inputCapture;
    InputType inputType;         // 输入数据类型，为enum{INVALID, CAMERA, VIDEO_FILE, IMAGE_LIST}
    bool goodInput;              // 是否为good标志位
    int flag;                    // 标定flag

private:
    string patternToUse;


};


//自定义的数据类型，要读写xml文件必须要在类内外定义相应的read和write函数，此处为类外函数定义，格式比较标准。
static inline void read(const FileNode& node, Settings& x, const Settings& default_value = Settings())
{
    if(node.empty())
        x = default_value;
    else
        x.read(node);
}

static inline void write(FileStorage& fs, const String&, const Settings& s )
{
    s.write(fs);
}

//检测、拍摄、标定？
enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };

bool runCalibrationAndSave(Settings& s, Size imageSize, Mat&  cameraMatrix, Mat& distCoeffs,
                           vector<vector<Point2f> > imagePoints );

int main(int argc, char* argv[])
{
    //输出help提示
    help();
    cout<<"\n\n";

    //! [file_read]
    Settings s;

  //  s.useFisheye = 1; //cheng

    const string inputSettingsFile = argc > 1 ? argv[1] : "default.xml";
    // Read the settings
    FileStorage fs(inputSettingsFile, FileStorage::READ);

    if (!fs.isOpened())
    {
        cout << "Could not open the configuration file: \"" << inputSettingsFile << "\"" << endl;
        return -1;
    }

    //将Settings标签写入s中
    fs["Settings"] >> s;
    // close Settings file
    fs.release();
    //! [file_read]

    //FileStorage fout("settings.yml", FileStorage::WRITE); // write config as YAML
    //fout << "Settings" << s;

    if (!s.goodInput)
    {
        cout << "Invalid input detected. Application stopping. " << endl;
        return -1;
    }

    //图像像素vector的vector，也就是图片
    vector<vector<Point2f> > imagePoints;
    //相机矩阵和内参
    Mat cameraMatrix, distCoeffs;
    //图像大小
    Size imageSize;


    //cout<<"s.inputType = "<<s.inputType<<endl;
    //cout<<"Settings::IMAGE_LIST = "<<Settings::IMAGE_LIST<<endl;

    //mode标志位？若输入数据类型为图片list置为CAPTURING，若不是则置为DETECTION
    int mode;
    if (s.inputType == Settings::IMAGE_LIST)
    {
        mode = CAPTURING;
    }else
    {
        mode = DETECTION;
    }

    //int mode = s.inputType == Settings::IMAGE_LIST ? CAPTURING : DETECTION;

    //时间戳
    clock_t prevTimestamp = 0;
    //颜色红和绿
    const Scalar RED(0,0,255), GREEN(0,255,0);
    //退出键
    const char ESC_KEY = 27;

    //鱼眼摄像头flag？
    cv::fisheye::CALIB_USE_INTRINSIC_GUESS;

    //! [get_input]
    int j =0;
    for(;;)
    {
        Mat view;
        bool blinkOutput = false;

        view = s.nextImage();

        //-----  If no more image, or got enough, then stop calibration and show result -------------
        //如果模式为CAPTURING，且图像已经大于或等于设定的用于标定的图像张数，则启动标定程序并保存
        if( mode == CAPTURING && imagePoints.size() >= (size_t)s.nrFrames )
        {
          if( runCalibrationAndSave(s, imageSize,  cameraMatrix, distCoeffs, imagePoints))
              mode = CALIBRATED;
          else
              mode = DETECTION;
        }

        if(view.empty())          // If there are no more images stop the loop
        {
            // if calibration threshold was not reached yet, calibrate now
            if( mode != CALIBRATED && !imagePoints.empty() )
                runCalibrationAndSave(s, imageSize,  cameraMatrix, distCoeffs, imagePoints);
            break;
        }
        //! [get_input]

        imageSize = view.size();  // Format input image.
        if( s.flipVertical )    flip( view, view, 0 );

        //! [find_pattern]
        vector<Point2f> pointBuf;

        bool found;

        int chessBoardFlags = CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE;

        if(!s.useFisheye) {
            // fast check erroneously fails with high distortions like fisheye
            chessBoardFlags |= CALIB_CB_FAST_CHECK;
        }

        switch( s.calibrationPattern ) // Find feature points on the input format
        {
        case Settings::CHESSBOARD:
            found = findChessboardCorners( view, s.boardSize, pointBuf, chessBoardFlags);
            break;
        case Settings::CIRCLES_GRID:
            found = findCirclesGrid( view, s.boardSize, pointBuf );
            break;
        case Settings::ASYMMETRIC_CIRCLES_GRID:
            found = findCirclesGrid( view, s.boardSize, pointBuf, CALIB_CB_ASYMMETRIC_GRID );
            break;
        default:
            found = false;
            break;
        }

        if (found)
        {
            cout<<"the "<<j<<" picture named "<<m[j]<<" found corner"<<endl;
            Mat copy_found = view.clone();
            imwrite("./found_corner/"+to_string(j)+".bmp", copy_found);
        } else
        {
            cout<<"the "<<j<<" picture named "<<m[j]<<" not found corner"<<endl;
            Mat copy_not_found = view.clone();
            imwrite("./not_found_corner/"+to_string(j)+".bmp", copy_not_found);
        }



        //! [find_pattern]
        //! [pattern_found]
        if ( found)                // If done with success,
        {
              // improve the found corners' coordinate accuracy for chessboard
                if( s.calibrationPattern == Settings::CHESSBOARD)
                {
                    Mat viewGray;
                    cvtColor(view, viewGray, COLOR_BGR2GRAY);
                    cornerSubPix( viewGray, pointBuf, Size(11,11),
                        Size(-1,-1), TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 30, 0.1 ));
                }

                if( mode == CAPTURING &&  // For camera only take new samples after delay time
                    (!s.inputCapture.isOpened() || clock() - prevTimestamp > s.delay*1e-3*CLOCKS_PER_SEC) )
                {
                    imagePoints.push_back(pointBuf);
                    prevTimestamp = clock();
                    blinkOutput = s.inputCapture.isOpened();
                }

                // Draw the corners.
                drawChessboardCorners( view, s.boardSize, Mat(pointBuf), found );
            Mat copy_found_with_corner = view.clone();
            imwrite("./found_with_corner/"+to_string(j)+".bmp", copy_found_with_corner);
        }

        j++;
        //! [pattern_found]
        //----------------------------- Output Text ------------------------------------------------
        //! [output_text]
        string msg = (mode == CAPTURING) ? "100/100" :
                      mode == CALIBRATED ? "Calibrated" : "Press 'g' to start";
        int baseLine = 0;
        Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);
        Point textOrigin(view.cols - 2*textSize.width - 10, view.rows - 2*baseLine - 10);

        if( mode == CAPTURING )
        {
            if(s.showUndistorsed)
                msg = format( "%d/%d Undist", (int)imagePoints.size(), s.nrFrames );
            else
                msg = format( "%d/%d", (int)imagePoints.size(), s.nrFrames );
        }

        putText( view, msg, textOrigin, 1, 1, mode == CALIBRATED ?  GREEN : RED);

        if( blinkOutput )
            bitwise_not(view, view);
        //! [output_text]
        //------------------------- Video capture  output  undistorted ------------------------------
        //! [output_undistorted]
        if( mode == CALIBRATED && s.showUndistorsed )
        {
            Mat temp = view.clone();
            undistort(temp, view, cameraMatrix, distCoeffs);
        }
        //! [output_undistorted]
        //------------------------------ Show image and check for input commands -------------------
        //! [await_input]
        imshow("Image View", view);
        //cout<<"\n"<<"s.inputCapture.isOpened() = "<<s.inputCapture.isOpened()<<endl;
        //char key = (char)waitKey(s.inputCapture.isOpened() ? 50 : s.delay);

        char key = (char)waitKey(100);

        //增加存储图片功能

        if( key  == ESC_KEY )
            break;

        if( key == 'u' && mode == CALIBRATED )
           s.showUndistorsed = !s.showUndistorsed;

        if(key == 'g' && s.inputCapture.isOpened() )
        {
            mode = CAPTURING;
            imagePoints.clear();
        }
        //! [await_input]
    }

    // -----------------------Show the undistorted image for the image list ------------------------
    //! [show_results]
    if( s.inputType == Settings::IMAGE_LIST && s.showUndistorsed )
    {
        Mat view, rview, map1, map2;

        if (s.useFisheye)
        {
            Mat newCamMat;
            fisheye::estimateNewCameraMatrixForUndistortRectify(cameraMatrix, distCoeffs, imageSize,
                                                                Matx33d::eye(), newCamMat, 1);

            fisheye::initUndistortRectifyMap(cameraMatrix, distCoeffs, Matx33d::eye(), newCamMat, imageSize,
                                             CV_16SC2, map1, map2);
        }
        else
        {
            initUndistortRectifyMap(
                cameraMatrix, distCoeffs, Mat(),
                getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0), imageSize,
                CV_16SC2, map1, map2);
        }

        for(size_t i = 0; i < s.imageList.size(); i++ )
        {
            view = imread(s.imageList[i], 1);

          //  imshow("Image View raw", view);

            if(view.empty())
                continue;
            remap(view, rview, map1, map2, INTER_LINEAR);
            imshow("Image View remap", rview);
            char c = (char)waitKey();
            if( c  == ESC_KEY || c == 'q' || c == 'Q' )
                break;
        }
    }
    //! [show_results]

    return 0;
}

//! [compute_errors]
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
//! [compute_errors]
//! [board_corners]
static void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners,
                                     Settings::Pattern patternType /*= Settings::CHESSBOARD*/)
{
    corners.clear();

    switch(patternType)
    {
    case Settings::CHESSBOARD:
    case Settings::CIRCLES_GRID:
        for( int i = 0; i < boardSize.height; ++i )
            for( int j = 0; j < boardSize.width; ++j )
                corners.push_back(Point3f(j*squareSize, i*squareSize, 0));
        break;

    case Settings::ASYMMETRIC_CIRCLES_GRID:
        for( int i = 0; i < boardSize.height; i++ )
            for( int j = 0; j < boardSize.width; j++ )
                corners.push_back(Point3f((2*j + i % 2)*squareSize, i*squareSize, 0));
        break;
    default:
        break;
    }
}
//! [board_corners]
static bool runCalibration( Settings& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                            vector<vector<Point2f> > imagePoints, vector<Mat>& rvecs, vector<Mat>& tvecs,
                            vector<float>& reprojErrs,  double& totalAvgErr)
{
    //! [fixed_aspect]
    //定义相机内参为3×3单位阵
    cameraMatrix = Mat::eye(3, 3, CV_64F);
    if( s.flag & CALIB_FIX_ASPECT_RATIO )
        cameraMatrix.at<double>(0,0) = s.aspectRatio;
    //! [fixed_aspect]

    //是否为鱼眼镜头，若为鱼眼镜头，8阶畸变，若不是则为4阶畸变。
    if (s.useFisheye) {
        distCoeffs = Mat::zeros(4, 1, CV_64F);
    } else {
        distCoeffs = Mat::zeros(8, 1, CV_64F);
    }

    vector<vector<Point3f> > objectPoints(1);
    calcBoardCornerPositions(s.boardSize, s.squareSize, objectPoints[0], s.calibrationPattern);

    objectPoints.resize(imagePoints.size(),objectPoints[0]);

    //Find intrinsic and extrinsic camera parameters
    double rms;

    if (s.useFisheye)
    {
        Mat _rvecs, _tvecs;
        rms = fisheye::calibrate(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, _rvecs, _tvecs, s.flag);


        double fx = cameraMatrix.at<double>(0,0);
        double cx = cameraMatrix.at<double>(0,2);
        double fy = cameraMatrix.at<double>(1,1);
        double cy = cameraMatrix.at<double>(1,2);
        double cc = cameraMatrix.at<double>(2,2);

        double r11 = _rvecs.at<double>(0,0);
        double r12 = _rvecs.at<double>(0,1);
        double r13 = _rvecs.at<double>(0,2);
        double r14 = _rvecs.at<double>(1,0);
        double r15 = _rvecs.at<double>(1,1);
        double r16 = _rvecs.at<double>(1,2);


        cout<<"\n\n\n";
        cout << "fx = "<<fx<<endl;
        cout << "cx = "<<cx<<endl;
        cout << "fy = "<<fy<<endl;
        cout << "cy = "<<cy<<endl;
        cout << "cc = "<<cc<<endl;
        cout << "r11 = "<<r11<<endl;
        cout << "r12 = "<<r12<<endl;
        cout << "r13 = "<<r13<<endl;
        cout << "r14 = "<<r14<<endl;
        cout << "r15 = "<<r15<<endl;
        cout << "r16 = "<<r16<<endl;

        rvecs.reserve(_rvecs.rows);
        tvecs.reserve(_tvecs.rows);
        for(int i = 0; i < int(objectPoints.size()); i++)
        {
            rvecs.push_back(_rvecs.row(i));
            tvecs.push_back(_tvecs.row(i));
        }

    } else
    {
        rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, s.flag);
    }

    cout << "Re-projection error reported by calibrateCamera: "<< rms << endl;

    bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

    totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints, rvecs, tvecs, cameraMatrix,
                                            distCoeffs, reprojErrs, s.useFisheye);

    return ok;
}

// Print camera parameters to the output file
//保存相机参数。参数就是一堆之前的setting和标定出来的数据。主要的功能就是将数据写入out_camera_data.xml文件。
static void saveCameraParams( Settings& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                              const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                              const vector<float>& reprojErrs, const vector<vector<Point2f> >& imagePoints,
                              double totalAvgErr )
{
    //链接xml文件，打开为写模式
    FileStorage fs( s.outputFileName, FileStorage::WRITE );

    //时间戳处理，用于紧接着的时间写入
    time_t tm;
    time( &tm );
    struct tm *t2 = localtime( &tm );
    char buf[1024];
    strftime( buf, sizeof(buf), "%c", t2 );

    //将时间写入xml文件中的<calibration_time>标签
    fs << "calibration_time" << buf;

    //如果旋转向量或重投影误差不为空，则将用于标定图片的数量写入<nr_of_frames>标签，
    if( !rvecs.empty() || !reprojErrs.empty() )
        fs << "nr_of_frames" << (int)std::max(rvecs.size(), reprojErrs.size());

    //一些直接写入的数据
    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;
    fs << "board_width" << s.boardSize.width;
    fs << "board_height" << s.boardSize.height;
    fs << "square_size" << s.squareSize;


    //下方为flag标签的写入，flag标签之前有一个comment，用cvWriteComment()进行书写
    if( s.flag & CALIB_FIX_ASPECT_RATIO )
        fs << "fix_aspect_ratio" << s.aspectRatio;

    if (s.flag)
    {
        if (s.useFisheye)
        {
            sprintf(buf, "flags:%s%s%s%s%s%s",
                     s.flag & fisheye::CALIB_FIX_SKEW ? " +fix_skew" : "",
                     s.flag & fisheye::CALIB_FIX_K1 ? " +fix_k1" : "",
                     s.flag & fisheye::CALIB_FIX_K2 ? " +fix_k2" : "",
                     s.flag & fisheye::CALIB_FIX_K3 ? " +fix_k3" : "",
                     s.flag & fisheye::CALIB_FIX_K4 ? " +fix_k4" : "",
                     s.flag & fisheye::CALIB_RECOMPUTE_EXTRINSIC ? " +recompute_extrinsic" : "");
        }
        else
        {
            sprintf(buf, "flags:%s%s%s%s",
                     s.flag & CALIB_USE_INTRINSIC_GUESS ? " +use_intrinsic_guess" : "",
                     s.flag & CALIB_FIX_ASPECT_RATIO ? " +fix_aspectRatio" : "",
                     s.flag & CALIB_FIX_PRINCIPAL_POINT ? " +fix_principal_point" : "",
                     s.flag & CALIB_ZERO_TANGENT_DIST ? " +zero_tangent_dist" : "");
        }
        //这里为xml文件写入注释,也就是输出文件中的这一句：
        //<!-- flags: +fix_skew +recompute_extrinsic -->
        cvWriteComment(*fs, buf, 0);
    }
    //写入flag标签数据
    fs << "flags" << s.flag;

    //同样，一堆写入。
    fs << "fisheye_model" << s.useFisheye;
    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;
    fs << "avg_reprojection_error" << totalAvgErr;
    if (s.writeExtrinsics && !reprojErrs.empty())
        fs << "per_view_reprojection_errors" << Mat(reprojErrs);

#if 0
/*
    if(s.writeExtrinsics && !rvecs.empty() && !tvecs.empty() )
    {
        CV_Assert(rvecs[0].type() == tvecs[0].type());
        Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());
        for( size_t i = 0; i < rvecs.size(); i++ )
        {
            Mat r = bigmat(Range(int(i), int(i+1)), Range(0,3));
            Mat t = bigmat(Range(int(i), int(i+1)), Range(3,6));

            CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
            CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
            //*.t() is MatExpr (not Mat) so we can use assignment operator
            r = rvecs[i].t();
            t = tvecs[i].t();
        }
        //cvWriteComment( *fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
        fs << "extrinsic_parameters" << bigmat;
    }
*/
    if(s.writePoints && !imagePoints.empty() )
    {
        Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
        for( size_t i = 0; i < imagePoints.size(); i++ )
        {
            Mat r = imagePtMat.row(int(i)).reshape(2, imagePtMat.cols);
            Mat imgpti(imagePoints[i]);
            imgpti.copyTo(r);
        }
        //最终的判定，将检测到的角点坐标矩阵写入。
        fs << "image_points" << imagePtMat;
    }
#endif

}

//! [run_and_save]
bool runCalibrationAndSave(Settings& s, Size imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                           vector<vector<Point2f> > imagePoints)
{
    vector<Mat> rvecs, tvecs;
    vector<float> reprojErrs;
    double totalAvgErr = 0;

    bool ok = runCalibration(s, imageSize, cameraMatrix, distCoeffs, imagePoints, rvecs, tvecs, reprojErrs,
                             totalAvgErr);
    cout << (ok ? "Calibration succeeded" : "Calibration failed")
         << ". avg re projection error = " << totalAvgErr << endl;

    if (ok)
        saveCameraParams(s, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, reprojErrs, imagePoints,
                         totalAvgErr);
    return ok;
}
//! [run_and_save]
