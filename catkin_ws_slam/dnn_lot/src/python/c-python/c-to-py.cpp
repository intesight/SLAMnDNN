#include "PyInterpreter.h"
#include "PyModule.h"

int main(int argc, char *argv[])
{
	int i;
	char buffer[100];
	InterpreterStatus status;
	PythonInterpreter pyInterpreter;
	PythonModule pyModule;

	// Initialize the Python Interpreter
	pyInterpreter.initialize();

	// Load python module
	// pyModule.loadModule(argv[1], argv[2]);
	pyModule.loadModule("inference_parking", "parking_detection");


	// Call functionPrepare the argument list for the call
	for (i=0; i<5; i++)
	{
		// Read image, covert to RGB format used by Python code
		cv::Mat img = cv::imread("./173.jpg", cv::IMREAD_COLOR);

        //图像前处理
        int INPUT_HEIGHT = 720;
        int INPUT_WIDTH  = 600;
        cv::Mat frame = cv::Mat::zeros(INPUT_HEIGHT, INPUT_WIDTH, CV_8UC3);
        cv::Mat imageROI= frame(cv::Rect((600-279), 0, 279, 720));
        img.copyTo(imageROI);


		// Create python Numpy object from image
		PyObject *pArray;
		status = pyInterpreter.getPyArgImage(frame, pArray);
		if (status != OK)
		{
			printf("Failed to create Python Numpy object from image!\n");
			return -1;
		}

		PyObject* pFunc = NULL;
		PyObject* pValue = NULL;

		// snprintf(buffer, 100, "%s%d", argv[3], i+1);
		// // Get function name object from string
		// status = pyInterpreter.getPyString(buffer, pFunc);
		
		// status = pyInterpreter.getPyString(argv[3], pFunc);
		status = pyInterpreter.getPyString("all", pFunc);

		if (status != OK)
		{
			printf("Failed to Get Python Function of a Class!\n");
			return -1;
		}


		// Call 4 different funtions of Python class with numpy array argument
		pValue = PyObject_CallMethodObjArgs(pyModule.getClass(), pFunc, pArray, NULL);


		if (pyModule.checkReturn(pValue) == OK)
		{
			// Convert function call return to Mat
			cv::Mat ret;
			status = pyInterpreter.getPyReturnMat(pValue, ret);
			if (status != OK)
			{
				printf("Failed to Get Return value as Mat!\n");
				return -1;
			}

			std::cout << ret << std::endl;
		}
		else
		{
			printf("Function call failed\n");
			pyInterpreter.printError();
			return -1;
		}

		// Remove reference to Python objects
		pyInterpreter.decRef(pFunc);
		pyInterpreter.decRef(pValue);

		// FIXME: Workaround to avoid failure of repeated function calls
		pyInterpreter.clearError();

		// Remove reference to Python object
		pyInterpreter.decRef(pArray);
		
	}

	// Clean up Python Module
	pyModule.cleanup();

	// Finish the Python Interpreter
	pyInterpreter.finalize();

	return 0;
}
