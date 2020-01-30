#ifndef __PYTHONCOMMON_H__
#define __PYTHONCOMMON_H__

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION

#include <Python.h>
#include <numpy/arrayobject.h>
#include <opencv2/opencv.hpp>

#if PY_MAJOR_VERSION >= 3
#define PYTHON_3X
#endif

// Python Bridge API status
enum InterpreterStatus
{
    OK = 0,
    FUNCTION_NOT_FOUND = -1,
    MODULE_NOT_FOUND = -2,
    RETURNED_NULL = -3,
    NOT_CALLABLE_OBJECT = -4,
    FUNCTION_CALL_FAILED = -5,
    UNKNOWN_ERROR = -6,
    ARGUMENT_ERROR = -7,
    UNKNOWN_INTEGER_ERROR = -8
};

#endif //__PYTHONCOMMON_H__
