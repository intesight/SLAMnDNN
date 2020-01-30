//#include "PyInterpreter.h"
//#include "PyModule.h"
#include "vehicle_detection.h"

int main(int argc, char *argv[])
{
	int i;
	char buffer[100];
	InterpreterStatus status;
	PythonInterpreter pyInterpreter;
	PythonModule pyModule;

	if (argc < 4)
	{
		printf("Usage: %s python_source class_name function_name\n", argv[0]);
		return 1;
	}

	// Initialize the Python Interpreter
	pyInterpreter.initialize();

	// Load python module
	//pyModule.loadModule(argv[1], argv[2]);
	vehicle_detection_init(pyModule);

	// Read image, covert to RGB format used by Sementic Segmentation Python code
	//cv::Mat img = cv::imread("outdoor_1.png", cv::IMREAD_COLOR);
	cv::Mat img = cv::imread("./car/2018-06-20 11-51-154.png", cv::IMREAD_COLOR);
	//cv::Mat img_rgb;
	//cv::cvtColor(img, img_rgb, cv::COLOR_BGR2RGB);
        
        //cv::imshow("car", img);
 	//cv::waitKey();

        cv::Mat vehicles;
	int rtn = vehicle_detection(pyInterpreter, pyModule, img, vehicles);
        //status = pyModule.callFunctionWithImage(pyInterpreter, argv[3], img, vehicles);

	std::cout << "Return = "<< std::endl << " "  << vehicles << std::endl << std::endl;

	// Create python Numpy object from image
	PyObject *pArray;
	status = pyInterpreter.getPyArgImage(img, pArray);
	if (status != OK)
	{
		printf("Failed to create Python Numpy object from image!\n");
		return -1;
	}

	PyObject* pFunc = NULL;
	PyObject* pValue = NULL;

	// Call functionPrepare the argument list for the call
	for (i=0; i<4; i++)
	{
		snprintf(buffer, 100, "%s%d", argv[3], i+1);

		// Get function name object from string
		status = pyInterpreter.getPyString(argv[3], pFunc);
		if (status != OK)
		{
			printf("Failed to Get Python Function of a Class!\n");
			return -1;
		}
		
		printf("\nSegmentation %d\n\n", i+1);

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

			std::cout << "Return = "<< std::endl << " "  << ret << std::endl << std::endl;

			//cv::Mat img_bgr;
			// Covert result to BGR format to save to file
			//cv::cvtColor(seg, img_bgr, cv::COLOR_RGB2BGR);

			//snprintf(buffer, 100, "%s%d%s", "outdoor_1_seg_", i+1, ".png");
			//cv::imwrite(buffer, img_bgr);

			printf("Return of call : %ld\n", PyLong_AsLong(pValue));
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
	}

	// Remove reference to Python object
	pyInterpreter.decRef(pArray);

	// Clean up Python Module
	pyModule.cleanup();

	// Finish the Python Interpreter
	pyInterpreter.finalize();

	return 0;
}
