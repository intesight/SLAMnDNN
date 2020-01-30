#PROJECT:slam
#author:cheng
#date: 2018/2/26
#
cmake_minimum_required(VERSION 3.5)
project(camera_calib)  # PROJECT name

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE  "-g -Wall -O3 ")

#add_subdirectory(view_convert)


# set(var value)
set(SOURCE_FILES main.cpp)

#set(OPENCV "/usr/local/") #opencv path
set(OpenCV_FOUND 1)
set(OpenCV_DIR /home/tangshuaishuai/opencv-3.1.0/build)
#set(OpenCV_DIR /home/chengguoqiang/cheng/opencv/opencv-3.1.0/build)
find_package(OpenCV REQUIRED)

ADD_DEFINITIONS(-DOPENCV)

#.h include path
include_directories(${OpenCV_INCLUDE_DIRS})

#target
add_executable(camera_calib ${SOURCE_FILES})

#tartet depend on lib
target_link_libraries(camera_calib ${OpenCV_LIBS})

#target_link_libraries(INTESIGHT_slam libmarker)
#target_link_libraries(INTESIGHT_slam libprobrd)