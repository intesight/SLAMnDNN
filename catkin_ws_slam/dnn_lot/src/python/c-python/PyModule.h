#ifndef __PYTHONMODULE_H__
#define __PYTHONMODULE_H__

#include "PyCommon.h"

class PythonModule
{
	private:
		
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
		
			std::cout << "PyModule Cleaned Up" << std::endl;
		}
		
		PyObject* getClass()
		{
			return m_CurrentClass;
		}
		
		InterpreterStatus loadModule(const std::string& module, const std::string& clazz)
		{
			if (m_CurrentModule == NULL)
			{
				std::cout << "Load Module" << std::endl;
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


		InterpreterStatus checkReturn(PyObject* o)
		{
			if (o == NULL)
				return FUNCTION_CALL_FAILED;
			else
				return OK;
		}
};

#endif //__PYTHONMODULE_H__
