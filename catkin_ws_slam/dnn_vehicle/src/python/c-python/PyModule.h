#ifndef __PYTHONMODULE_H__
#define __PYTHONMODULE_H__

#include "PyCommon.h"

class PythonModule
{
	private:
		std::string name;
		PyObject * m_CurrentModuleName = NULL;
		PyObject * m_CurrentModule = NULL;
		PyObject * m_CurrentTable = NULL;
		PyObject * m_CurrentClassName = NULL;
		PyObject * m_CurrentClass = NULL;

	public:
		PythonModule()
		{
		}

		~PythonModule()
		{
		}
		
		void cleanup()
		{
			if (m_CurrentClassName != NULL)
				Py_DECREF(m_CurrentClassName);
			if (m_CurrentClass != NULL)
				Py_DECREF(m_CurrentClass);
			if (m_CurrentTable != NULL)
				Py_DECREF(m_CurrentTable);
			if (m_CurrentModule != NULL)
				Py_DECREF(m_CurrentModule);
			if (m_CurrentModuleName != NULL)
				Py_DECREF(m_CurrentModule);
		
			std::cout << "PyModule " << name << " Cleaned Up" << std::endl;
		}
		
		PyObject* getClass()
		{
			return m_CurrentClass;
		}
		
		InterpreterStatus loadModule(const std::string& module, const std::string& clazz)
		{
			if (m_CurrentModule == NULL)
			{
				name = module;
				std::cout << "Load Module: " << module << std::endl;
#ifdef PYTHON_3X
				m_CurrentModuleName = PyUnicode_FromString(module.c_str());
#else
				m_CurrentModuleName = PyString_FromString(module.c_str());
#endif
				// Load the module object
				m_CurrentModule = PyImport_Import(m_CurrentModuleName);
				if(!m_CurrentModule)
					return MODULE_NOT_FOUND;

				// pDict is a borrowed reference
				m_CurrentTable = PyModule_GetDict(m_CurrentModule);

				// Build the name of a callable class 
				m_CurrentClassName = PyDict_GetItemString(m_CurrentTable, clazz.c_str());

				// Create an instance of the class
				if (PyCallable_Check(m_CurrentClassName))
				{
					m_CurrentClass = PyObject_CallObject(m_CurrentClassName, NULL);
				}

				return OK;
			}
		}

		InterpreterStatus callFunctionWithImage(PythonInterpreter& pyInterpreter, const std::string& func, cv::Mat& input, cv::Mat& output)
		{
			InterpreterStatus status;
			PyObject* pFunc = NULL;
			PyObject* pValue = NULL;

			// Create python Numpy object from image 
			PyObject *pArray;
			status = pyInterpreter.getPyArgImage(input, pArray);
			if (status != OK)
			{
				printf("Failed to create Python Numpy object from image!\n");
				return UNKNOWN_ERROR;
			}

			// Get function name object from string
			status = pyInterpreter.getPyString(func, pFunc);
			if (status != OK)
			{
				printf("Failed to Get Python Function of a Class!\n");
				return UNKNOWN_ERROR;
			}

			// Call function of Python class with numpy array argument
			pValue = PyObject_CallMethodObjArgs(m_CurrentClass, pFunc, pArray, NULL);

			if (checkReturn(pValue) == OK)
 			{
				// Convert function call return to Mat
				status = pyInterpreter.getPyReturnMat(pValue, output);

				if (status != OK)
				{
					printf("Failed to Get Return value as Mat!\n");
					return RETURNED_NULL;
        			}
    			}
			else
			{
 				printf("Function call %s failed\n", func.c_str());
				pyInterpreter.printError();
 				return FUNCTION_CALL_FAILED;
			}

			// Remove reference to Python object
			pyInterpreter.decRef(pArray);

			// FIXME: Workaround to avoid failure of repeated function calls
			pyInterpreter.clearError();

			return OK;
		}
		
		InterpreterStatus checkReturn(PyObject* o)
		{
			if (o == NULL)
				return FUNCTION_CALL_FAILED;
			else
				return OK;
		}
};

#endif //__PYTHONMODULE_H__
