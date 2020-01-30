

g++ c-to-py.cpp -g -o c-to-py -fpermissive -I/usr/include/python2.7 -L/usr/lib/python2.7/config-x86_64-linux-gnu/ -L/usr/local/lib \
  -lpython2.7 -lopencv_core -lopencv_imgproc -lopencv_imgcodecs

