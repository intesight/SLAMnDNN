#/bin/bash

cmake .. -G"Unix Makefiles"
make
cp *.so ..
#cp *.a ..
