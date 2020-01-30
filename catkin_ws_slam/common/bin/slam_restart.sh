#!/bin/bash

SCRIPT=$( readlink -m $( type -p $0 ))
BASE_DIR=`dirname ${SCRIPT}`

export DISPLAY=:0
export PATH="$PATH:/usr/local/cuda/bin"
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/usr/local/cuda/lib64/"
export LIBRARY_PATH="$LIBRARY_PATH:/usr/local/cuda/lib64"

source /opt/ros/kinetic/setup.bash
source ~/rosbuild_ws/setup.bash
source ~/rosbuild_ws_left/setup.bash
source ~/rosbuild_ws_left/setup.bash
source ~/rosbuild_ws_back/setup.bash
ulimit -c unlimited 

rosnode kill /viewer_right
rosnode kill /LSD_SLAM
rosnode kill /LSD_SLAM_back
rosnode kill /LSD_SLAM_left
rosnode kill /LSD_SLAM_right

$BASE_DIR/front_lsd_slam.sh &
$BASE_DIR/back_lsd_slam.sh &
$BASE_DIR/left_lsd_slam.sh &
$BASE_DIR/right_lsd_slam.sh &
