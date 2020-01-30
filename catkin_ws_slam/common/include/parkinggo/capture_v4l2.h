#ifndef __CAPTURE_V4L2_H_
#define __CAPTURE_V4L2_H_


#include <opencv2/videoio.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
using namespace cv;

int capture_v4l2 ();
Mat read_frame (void);

#endif