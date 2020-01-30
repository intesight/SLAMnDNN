#!/bin/bash

SCRIPT=$( readlink -m $( type -p $0 ))
BASE_DIR=`dirname ${SCRIPT}`
CFG_DIR=$BASE_DIR/../cfg

cd ~/rosbuild_ws_right/
source ~/rosbuild_ws_right/setup.sh

#rosrun lsd_slam_viewer viewer_right &

rosrun lsd_slam_core live_slam_right image:=/raw_right _calib:=$CFG_DIR/cameraCalibration_half_amba.cfg &


