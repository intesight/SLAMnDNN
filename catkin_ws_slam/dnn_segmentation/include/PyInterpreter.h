#ifndef __PYTHONINTERPRETER_H__
#define __PYTHONINTERPRETER_H__

#include "PyCommon.h"

class PythonInterpreter
{
    public:
        // Constructor & Destructor
        PythonInterpreter();
        ~PythonInterpreter();
               
        // Intialize Python Interpreter
	    void initialize();

        // Finalize Python Interpreter
        void finalize();

        // Return if Python Interpreter is initialized
        bool isInitialized();

        /****
          Create a Python String object from a c++ string
            Args:
              INPUT  const std::string s, input string
              OUPUT  PyObject*& o, Created Python String object
            Return   InterpreterStatus
        ****/
        InterpreterStatus getPyString(const std::string s, PyObject*& o);

        /****
          Create a Python Long object from a c++ long number
            Args:
              INPUT  const long l, input long number
              OUPUT  PyObject*& o, Created Python Long object
            Return   InterpreterStatus
        ****/
        InterpreterStatus getPyLong(const long l, PyObject*& o);

        /****
          Create a Python Float object from a c++ double number
            Args:
              INPUT  const double d, input double number
              OUPUT  PyObject*& o, Created Python Float object
            Return   InterpreterStatus
        ****/
        InterpreterStatus getPyFloat(const double d, PyObject*& o);

        /****
          Create a Python empty List object
            Args:
              OUPUT  PyObject*& o, Created Python List object
            Return   InterpreterStatus
        ****/
        InterpreterStatus getPyList(PyObject*& o);

        /****
          Append Item to a Python List object
            Args:
              INPUT  PyObject*& o, Python List object to append
              INPUT  PyObject*&, Item to be appended
            Return   InterpreterStatus
        ****/
        InterpreterStatus pyListAppend(PyObject*& o, PyObject*& i);

        /****
          Create a Python Numpy object from a OpenCV Mat image
            Args:
              INPUT  const cv::Mat& image, input OpenCV Mat image
              OUPUT  PyObject*& o, Created Python Numpy object
            Return   InterpreterStatus
        ****/
        InterpreterStatus getPyArgImage(const cv::Mat& image, PyObject*& o);

        /****
          Get OpenCV Mat from Python Numpy object
            Args:
              INPUT  const cv::Mat& image, input OpenCV Mat image
              OUPUT  PyObject*& o, Created Python Numpy object
            Return   InterpreterStatus
        ****/
        InterpreterStatus getPyReturnMat(PyObject*& o, cv::Mat& m);

        /****
          Decrease reference of Python Object
            Args:
              INPUT  PyObject*& o, Python Object
        ****/
        void decRef(PyObject*& o);

        // Print Python API error
        void printError();

        // Clear Python API error
        void clearError();

    private:
        // private variable indicates if Python Interpreter is initialized
        bool m_Initialized;

};

#endif //__PYTHONINTERPRETER_H__
