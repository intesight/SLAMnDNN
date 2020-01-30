#ifndef __PYTHONMODULE_H__
#define __PYTHONMODULE_H__

#include "PyCommon.h"
#include "PyInterpreter.h"

/****
  Wrapper class for Python class
****/
class PythonModule
{
    public:
        // Constructor & Destructor
        PythonModule();
        ~PythonModule();

        // Cleanup Python Module	
        void cleanup();

        /****
          Get object of this class
            Return   class instance
        ****/
        PyObject* getClass();
		
        /****
          Load a Python module
            Args: 
              INPUT  const std::string& module,  name of a Python file (without .py suffix, e.g. inference.py --> inference)
              INPUT  const std::string& clazz,   name of Python class defined in module
            Return   InterpreterStatus
        ****/
        InterpreterStatus loadModule(const std::string& module, const std::string& clazz);

        /****
          Call a method of a Python class with image input
            Args: 
              INPUT  PythonInterpreter& pyInterpreter
              INPUT  const std::string& func, method name of Python class defined in module
              INPUT  cv::Mat& image, input OpenCV image
              OUTPUT cv::Mat& output, output OpenCV Mat array
            Return   ImterpreterStatus
        ****/
        InterpreterStatus callFunctionWithImage(PythonInterpreter& pyInterpreter, const std::string& func, cv::Mat& image, cv::Mat& output);
		
        /****
          Call a method of a Python class with image and List input
            Args: 
              INPUT  PythonInterpreter& pyInterpreter
              INPUT  const std::string& func, method name of Python class defined in module
              INPUT  cv::Mat& image, input OpenCV image
              INPUT  PyObject* list, input Python List
              OUTPUT cv::Mat& output, output OpenCV Mat array
            Return   ImterpreterStatus
        ****/
        InterpreterStatus callFunctionWithImageAndList(PythonInterpreter& pyInterpreter, const std::string& func, cv::Mat& image, PyObject*& list, cv::Mat& output);
		
        /****
          Check if return of Python function call is valid
            Args: 
              INPUT  PyObject* o,  return of Python function call
            Return   InterpreterStatus
        ****/
        InterpreterStatus checkReturn(PyObject* o);

    private:
        // private variable of Python script name
        std::string name;

        // private variables of Python objects
        PyObject * m_CurrentModuleName = NULL;
        PyObject * m_CurrentModule = NULL;
        PyObject * m_CurrentTable = NULL;
        PyObject * m_CurrentClassName = NULL;
        PyObject * m_CurrentClass = NULL;
};

#endif //__PYTHONMODULE_H__
