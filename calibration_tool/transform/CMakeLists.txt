cmake_minimum_required(VERSION 3.5)
project(calib_model_lut)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

SET(CMAKE_BUILD_TYPE "debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

#if (${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm")
#  set(LIB_ARCH ${S32V_ROOT_DIR}/usr/lib/aarch64-linux-gnu)
#endif ()

set(OpenCV_DIR /usr/local/share/OpenCV)
find_package(OpenCV 3 REQUIRED)

#set(SOURCE_FILES main.cpp cam_model.cpp image_view_convert.cpp)
#include_directories(${PROJECT_SOURCE_DIR})

set(SOURCE_FILES main.cpp)

#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib/${CMAKE_SYSTEM_PROCESSOR})

include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(
  ${PROJECT_SOURCE_DIR}
  ${LIB_ARCH}
)

aux_source_directory(. DIR_SRCS)

#add_library(lut SHARED ${DIR_SRCS})

add_library(lut SHARED cam_model.cpp image_view_convert.cpp)
#add_library(lut STATIC cam_model.cpp image_view_convert.cpp)

add_executable(calib_model_lut ${SOURCE_FILES})

target_link_libraries(calib_model_lut 
  ${OpenCV_LIBS} 
  lut
)
