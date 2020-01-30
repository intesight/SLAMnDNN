#include "PyInterpreter.h"
#include "PyModule.h"

void vehicle_detection_init(PythonModule& pyModule)
{
    pyModule.loadModule("squdet", "inference");
}

void vehicle_detection_stop(PythonModule& pyModule)
{
    pyModule.cleanup();
}

int vehicle_detection(PythonInterpreter& pyInterpreter, PythonModule& pyModule, cv::Mat& input, cv::Mat& output)
{
    InterpreterStatus status = pyModule.callFunctionWithImage(pyInterpreter, "predict", input, output);

    if (status != OK)
    {
        printf("Function call failed\n");
        return -1;
    }

    return 0;
}

