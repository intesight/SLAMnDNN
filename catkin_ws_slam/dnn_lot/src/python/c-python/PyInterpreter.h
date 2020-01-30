#ifndef __PYTHONINTERPRETER_H__
#define __PYTHONINTERPRETER_H__

#include "PyCommon.h"

class PythonInterpreter
{
	private:
		bool m_Initialized;

	public:
		PythonInterpreter()
		{
			m_Initialized = false;
		}

		~PythonInterpreter()
		{
		}

		void initialize()
		{
			if (!m_Initialized)
			{
				Py_Initialize();
				m_Initialized = true;
				std::cout << "PythonInterpreter initialized" << std::endl;
			}
		}

		void finalize()
		{
			// Finish the Python Interpreter
			if(m_Initialized)
			{
				std::cout << "PythonInterpreter finalized" << std::endl;
				Py_Finalize();
			}
		}

		bool isInitialized()
		{
			return m_Initialized;
		}


		InterpreterStatus getPyString(const std::string s, PyObject*& o)
		{
#ifdef PYTHON_3X
			o = PyUnicode_FromString(s.c_str());
#else
			o = PyString_FromString(s.c_str());
#endif
			if (!o)
				return UNKNOWN_ERROR;

			return OK;
		}

		InterpreterStatus getPyArgImage(cv::Mat& image, PyObject*& o)
		{
			// Build the array in C++
			npy_intp dims[3];
			dims[0] = image.rows;
			dims[1] = image.cols;
			dims[2] = image.channels();

			// Convert it to a NumPy array.
			import_array1(UNKNOWN_ERROR);
			o = PyArray_SimpleNewFromData(3, dims, NPY_UBYTE, reinterpret_cast<void*>(image.data));
			if (!o)
			{
				std::cout << "PyArray_SimpleNewFromData failed" << std::endl;
				return UNKNOWN_ERROR;
			}

			return OK;
		}

		void decRef(PyObject*& o)
		{
			if (o)
				Py_DECREF(o);
		}

		void printError()
		{
			PyErr_Print();
		}

		void clearError()
		{
			PyErr_Clear();
		}

		InterpreterStatus getPyReturnMat(PyObject*& o, cv::Mat& m)
		{
			if( !PyArray_Check(o) )
			{
				std::cout << "Object is not a numpy array" << std::endl;
				return UNKNOWN_ERROR;
			}

			PyArrayObject* oarr = (PyArrayObject*) o;

			bool needcopy = false, needcast = false;
			int typenum = PyArray_TYPE(oarr), new_typenum = typenum;
			int type = typenum == NPY_UBYTE ? CV_8U :
					typenum == NPY_BYTE ? CV_8S :
					typenum == NPY_USHORT ? CV_16U :
					typenum == NPY_SHORT ? CV_16S :
					typenum == NPY_INT ? CV_32S :
					typenum == NPY_INT32 ? CV_32S :
					typenum == NPY_FLOAT ? CV_32F :
					typenum == NPY_DOUBLE ? CV_64F : -1;

			if( type < 0 )
			{
				if( typenum == NPY_INT64 || typenum == NPY_UINT64 || typenum == NPY_LONG )
				{
					needcopy = needcast = true;
					new_typenum = NPY_INT;
					type = CV_32S;
				}
				else
				{
					std::cout << "data type = " << typenum << " is not supported" << std::endl;
					return UNKNOWN_ERROR;
				}
			}

#ifndef CV_MAX_DIM
			const int CV_MAX_DIM = 32;
#endif

			int ndims = PyArray_NDIM(oarr);
			if(ndims >= CV_MAX_DIM)
			{
				std::cout << "Dimensionality (=" << ndims << ") is too high" << std::endl;
				return UNKNOWN_ERROR;
			}

			int size[CV_MAX_DIM+1];
			size_t step[CV_MAX_DIM+1];
			size_t elemsize = CV_ELEM_SIZE1(type);
			const npy_intp* _sizes = PyArray_DIMS(oarr);
			const npy_intp* _strides = PyArray_STRIDES(oarr);
			bool ismultichannel = ndims == 3 && _sizes[2] <= CV_CN_MAX;

			for( int i = ndims-1; i >= 0 && !needcopy; i-- )
			{
				// these checks handle cases of
				//  a) multi-dimensional (ndims > 2) arrays, as well as simpler 1- and 2-dimensional cases
				//  b) transposed arrays, where _strides[] elements go in non-descending order
				//  c) flipped arrays, where some of _strides[] elements are negative
				// the _sizes[i] > 1 is needed to avoid spurious copies when NPY_RELAXED_STRIDES is set
				if( (i == ndims-1 && _sizes[i] > 1 && (size_t)_strides[i] != elemsize) ||
						(i < ndims-1 && _sizes[i] > 1 && _strides[i] < _strides[i+1]) )
					needcopy = true;
			}

			if( ismultichannel && _strides[1] != (npy_intp)elemsize*_sizes[2] )
				needcopy = true;

			if (needcopy)
			{
				if( needcast )
				{
					o = PyArray_Cast(oarr, new_typenum);
					oarr = (PyArrayObject*) o;
				}
				else
				{
					oarr = PyArray_GETCONTIGUOUS(oarr);
					o = (PyObject*) oarr;
				}

				_strides = PyArray_STRIDES(oarr);
			}

		// Normalize strides in case NPY_RELAXED_STRIDES is set
		size_t default_step = elemsize;
		for ( int i = ndims - 1; i >= 0; --i )
		{
			size[i] = (int)_sizes[i];
			if ( size[i] > 1 )
			{
				step[i] = (size_t)_strides[i];
				default_step = step[i] * size[i];
			}
			else
			{
				step[i] = default_step;
				default_step *= size[i];
			}
		}

		// handle degenerate case
		if( ndims == 0)
		{
			size[ndims] = 1;
			step[ndims] = elemsize;
			ndims++;
		}

		if( ismultichannel )
		{
			ndims--;
			type |= CV_MAKETYPE(0, size[2]);
		}

		m = cv::Mat(ndims, size, type, PyArray_DATA(oarr), step);
		//m.u = g_numpyAllocator.allocate(o, ndims, size, type, step);
		//m.addref();

		if( !needcopy )
		{
			Py_INCREF(o);
		}
		//m.allocator = &g_numpyAllocator;

		return OK;
	}
};

#endif //__PYTHONINTERPRETER_H__
