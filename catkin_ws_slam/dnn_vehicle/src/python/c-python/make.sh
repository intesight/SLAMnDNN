#g++ c-to-py.cpp -g -o c-to-py -fpermissive -I/usr/include/python3.5 -L/usr/lib/python3.5/config-3.5m-x86_64-linux-gnu/ \
#   -lpython3.5 -lopencv_core -lopencv_imgproc -lopencv_imgcodecs

g++ c-to-py.cpp -g -o c-to-py -fpermissive -I../include -I/usr/include/python2.7 -L/usr/lib/python2.7/config-x86_64-linux-gnu/ \
   -lpython2.7 -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -lopencv_highgui
