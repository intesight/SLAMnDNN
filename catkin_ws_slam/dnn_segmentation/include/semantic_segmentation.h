#include "PyInterpreter.h"
#include "PyModule.h"

/****
  Intialize Semantic Segmentation module
    Args
      INPUT  PythonModule& pyModule, Python module
****/
void semantic_segmentation_init(PythonModule& pyModule)
{
    pyModule.loadModule("semantic-segmentation", "segmentation");
}

/****
  Stop Semantic Segmentation module
    Args
      INPUT  PythonModule& pyModule, Python module
****/
void semantic_segmentation_stop(PythonModule& pyModule)
{
    pyModule.cleanup();
}

/****
  Semantic segment one image
    Args
      INPUT  PythonInterpreter& pyInterpreter, Python interpreter
      INPUT  PythonModule& pyModule, Python module
      INPUT  cv::Mat& input, input OpenCV BGR image
      OUTPUT cv::Mat& output, output OpenCV BGR image
    Return
      0      success
      -1     failed
****/
int semantic_segmentation(PythonInterpreter& pyInterpreter, PythonModule& pyModule, cv::Mat& input, cv::Mat& output)
{
    InterpreterStatus status;

    // Covert to RGB format used by Sementic Segmentation Python code
    cv::Mat img_rgb;
    cv::cvtColor(input, img_rgb, cv::COLOR_BGR2RGB);

    cv::Mat img;
    status = pyModule.callFunctionWithImage(pyInterpreter, "segment", input, img);

    if (status != OK)
    {
        printf("Function call failed\n");
        return -1;
    }

    // Covert result to BGR format to save to file
    cv::cvtColor(img, output, cv::COLOR_RGB2BGR);

    return 0;
}

