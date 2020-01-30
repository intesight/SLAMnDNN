#!/bin/bash

SCRIPT=$( readlink -m $( type -p $0 ))
BASE_DIR=`dirname ${SCRIPT}`
PARKINGGO_BIN_DIR=$BASE_DIR/../bin

source $BASE_DIR/../setup.sh

rosnode kill /viewer_right
rosnode kill /LSD_SLAM
rosnode kill /LSD_SLAM_back
rosnode kill /LSD_SLAM_left
rosnode kill /LSD_SLAM_right

rosnode kill /dnn_segmentation
rosnode kill /dnn_vehicle
rosnode kill /dnn_lot
rosnode kill /comm_mcu
rosnode kill /cam_capture

$PARKINGGO_BIN_DIR/stop_process.sh rosmaster
$PARKINGGO_BIN_DIR/stop_process.sh roscore
$PARKINGGO_BIN_DIR/stop_process.sh rosout
$PARKINGGO_BIN_DIR/stop_process.sh ./video

rm ~/.ros/*.pid

echo Press enter to continue; read dummy;

