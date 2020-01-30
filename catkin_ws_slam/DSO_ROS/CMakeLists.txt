cmake_minimum_required(VERSION 2.4.6)
project(dso_ros)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  roscpp
  sensor_msgs
  cv_bridge
  message_generation
)

OPTION(__RUN_IN_EMBEDDED_SYSTEM__ "Run in embedded s32v " OFF)

if(__RUN_IN_EMBEDDED_SYSTEM__)
add_definitions("-DCMAKE_TOOLCHAIN_FILE=~/S32V_Cross_Compiling/s32v_compile/rostoolchain.cmake")
add_definitions("-D__RUN_IN_EMBEDDED_SYSTEM__")


if (${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm")
  set(LIB_ARCH ${S32V_ROOT_DIR}/usr/lib/aarch64-linux-gnu)
endif ()

endif(__RUN_IN_EMBEDDED_SYSTEM__)

set(DSO_PATH /home/intesight/dso)
message("\n\n---- LOOKING FOR DSO at\n\"${DSO_PATH}\"")

find_package(Pangolin 0.2 REQUIRED)
message("\n\n---- FOUND Pangolin Headers at\n\"${Pangolin_INCLUDE_DIRS}\"")
message("---- FOUND Pangolin Libs at\n\"${Pangolin_LIBRARIES}\"")

find_package(OpenCV REQUIRED)
message("\n\n---- FOUND OpenCV Headers at\n\"${OpenCV_INCLUDE_DIRS}\"")
message("---- FOUND OpenCV Libs at\n\"${OpenCV_LIBS}\"")

find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system thread)
find_package(Threads QUIET)

OPTION(__SAVE_MAP_AS_PCD__ "Save the generated map into pcd format" ON)

if(__SAVE_MAP_AS_PCD__)
  add_definitions("-D__SAVE_MAP_AS_PCD__")
  # pcl 
  find_package( PCL REQUIRED COMPONENT common io )
  include_directories(
    include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} )
  add_definitions( ${PCL_DEFINITIONS} )
endif()

if(__RUN_IN_EMBEDDED_SYSTEM__)
    set(DSO_LIBRARY /home/liulifeng/S32V_Cross_Compiling/dso/build/lib/libdso.a)
else()
    find_library(DSO_LIBRARY dso ${DSO_PATH}/build/lib)
endif()

message("\n\n---- LOOKING FOR DSO_LIBRARY at\n\"${DSO_LIBRARY}\"")

# //cheng debug
 add_definitions("-D__DEBUG_USING_INTESIGHT_INPUT__")
 add_definitions("-DDEBUG_USING_INTESIGHT_INPUT")

#add_definitions("-D__USE_IMAGE_FILES__")

add_message_files(
  FILES
  keypointDSO.msg)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)
catkin_package(
  CATKIN_DEPENDS
    message_runtime
    geometry_msgs
    roscpp
    sensor_msgs
    cv_bridge
    std_msgs
)

###########
## Build ##
###########

if(__RUN_IN_EMBEDDED_SYSTEM__)
set(CMAKE_CXX_FLAGS
   "${SSE_FLAGS} -O2 -g -std=c++0x -fno-omit-frame-pointer"
)
else()
set(CMAKE_CXX_FLAGS
   "${SSE_FLAGS} -O2 -g -std=c++0x -march=native -fno-omit-frame-pointer"
)
endif()

include_directories(
  ${PROJECT_SOURCE_DIR}/src
  ${DSO_PATH}/src
  ${DSO_PATH}/src/FullSystem
  ${DSO_PATH}/src/util
  ${DSO_PATH}/src/IOWrapper
  ${DSO_PATH}/thirdparty/Sophus
  ${Pangolin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

link_directories(${DSO_PATH}/thirdparty/lut)


## Declare a C++ executable
add_executable(dso_live src/dso_live.cpp)
target_link_libraries(dso_live
    ${DSO_LIBRARY}
    ${Pangolin_LIBRARIES}
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    lut
    )
if(__SAVE_MAP_AS_PCD__)
  target_link_libraries(dso_live ${PCL_LIBRARIES})
endif()

# # 不使用ROS,直接从图像数据输入
# add_executable(useImageFiles
#   src/useImageFiles.cpp
# )
# add_dependencies(useImageFiles ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(useImageFiles
#     ${DSO_LIBRARY}
#     ${Pangolin_LIBRARIES}
#     ${OpenCV_LIBS}
#     ${catkin_LIBRARIES}
#     lut
# )


# # 在嵌入式开发板中运行,去除pangolin依赖
# add_executable(runInEmbedded
#   src/runInEmbedded.cpp
# )
# add_dependencies(runInEmbedded ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(runInEmbedded
#     ${DSO_LIBRARY}
#     ${Pangolin_LIBRARIES}
#     ${OpenCV_LIBS}
#     ${catkin_LIBRARIES}
#     lut
# )

# if(__SAVE_MAP_AS_PCD__)
#   target_link_libraries(dso_live ${PCL_LIBRARIES})
#   target_link_libraries(useImageFiles ${PCL_LIBRARIES})
#   target_link_libraries(runInEmbedded ${PCL_LIBRARIES})
# endif()

add_executable(dso_traj_vis_pub
  src/dso_traj_vis_pub.cpp
)
add_dependencies(dso_traj_vis_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dso_traj_vis_pub
  ${catkin_LIBRARIES}
)

