#!/bin/bash

SCRIPT=$( readlink -m $( type -p $0 ))
BASE_DIR=`dirname ${SCRIPT}`
CFG_DIR=$BASE_DIR/../cfg

cd ~/rosbuild_ws_left/
source ~/rosbuild_ws_left/setup.sh

rosrun lsd_slam_core live_slam_left image:=/raw_left _calib:=$CFG_DIR/cameraCalibration_half_amba.cfg &

