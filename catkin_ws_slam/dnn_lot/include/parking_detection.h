#include "PyInterpreter.h"
#include "PyModule.h"

void parking_detection_init(PythonModule& pyModule)
{
    pyModule.loadModule("inference_parking", "parking_detection");
}

void parking_detection_stop(PythonModule& pyModule)
{
    pyModule.cleanup();
}

int parking_detection(PythonInterpreter& pyInterpreter, PythonModule& pyModule, cv::Mat& input, cv::Mat& output, bool is_detect = true)
{
    InterpreterStatus status;

    if (is_detect)
        status = pyModule.callFunctionWithImage(pyInterpreter, "detect_one", input, output);
    else
        status = pyModule.callFunctionWithImage(pyInterpreter, "track_one", input, output);

    if (status != OK)
    {
        printf("Function call failed\n");
        return -1;
    }

    return 0;
}

