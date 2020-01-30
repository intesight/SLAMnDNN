cmake_minimum_required(VERSION 2.8)  
project(readdata)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(OpenCV_FOUND 1)
#set(OpenCV_DIR /usr/local/share/OpenCV)
set(OpenCV_DIR/home/wangkun/opencv-3.1.0/build)

find_package(OpenCV REQUIRED)
ADD_DEFINITIONS(-DOPENCV)

include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(calib_extrinstic readdata.cpp fisheye.cpp)
target_link_libraries(calib_extrinstic ${OpenCV_LIBS})
