#!/bin/bash

SCRIPT=$( readlink -m $( type -p $0 ))
BASE_DIR=`dirname ${SCRIPT}`
CFG_DIR=$BASE_DIR/../cfg

cd ~/rosbuild_ws/
source ~/rosbuild_ws/setup.sh

rosrun lsd_slam_core live_slam image:=/raw_front _calib:=$CFG_DIR/cameraCalibration_half_amba.cfg &

