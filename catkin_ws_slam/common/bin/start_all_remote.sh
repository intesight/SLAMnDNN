#!/bin/bash

SCRIPT=$( readlink -m $( type -p $0 ))
BASE_DIR=`dirname ${SCRIPT}`
PARKINGGO_DIR=$BASE_DIR/../

echo "Start in " $PARKINGGO_DIR

PARKINGGO_CFG=$PARKINGGO_DIR/cfg/parkinggo_config.xml

export DISPLAY=:0
export PATH="$PATH:/usr/local/cuda/bin"
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/usr/local/cuda/lib64/"
export LIBRARY_PATH="$LIBRARY_PATH:/usr/local/cuda/lib64"

export TF_CPP_MIN_LOG_LEVEL=2

source /opt/ros/kinetic/setup.bash
source ~/rosbuild_ws/setup.bash
source ~/rosbuild_ws_left/setup.bash
source ~/rosbuild_ws_left/setup.bash
source ~/rosbuild_ws_back/setup.bash
ulimit -c unlimited

source $PARKINGGO_DIR/bin/python.env.sh
echo $PYTHONPATH

roscore &

sleep 1

source $PARKINGGO_DIR/setup.sh

cd $PARKINGGO_DIR

rosrun cam_capture cam_capture &

enable=$(xmllint --xpath "string(//opencv_storage/mcu_enable)" $PARKINGGO_CFG)
if [ $enable -eq 1 ] 
  then
    rosrun comm_mcu comm_mcu &
fi

enable=$(xmllint --xpath "string(//opencv_storage/parking_detection)" $PARKINGGO_CFG)
if [ $enable -eq 1 ] 
  then
    rosrun dnn_lot dnn_lot &
fi

enable=$(xmllint --xpath "string(//opencv_storage/semantic_segmentation)" $PARKINGGO_CFG)
if [ $enable -eq 1 ] 
  then
    rosrun dnn_segmentation dnn_segmentation &
fi

enable=$(xmllint --xpath "string(//opencv_storage/vehicle_detection)" $PARKINGGO_CFG)
if [ $enable -eq 1 ] 
  then
    rosrun dnn_vehicle dnn_vehicle &
fi

enable=$(xmllint --xpath "string(//opencv_storage/front_slam)" $PARKINGGO_CFG)
if [ $enable -eq 1 ] 
  then
    $PARKINGGO_DIR/bin/front_lsd_slam.sh &
fi

enable=$(xmllint --xpath "string(//opencv_storage/back_slam)" $PARKINGGO_CFG)
if [ $enable -eq 1 ] 
  then
    $PARKINGGO_DIR/bin/back_lsd_slam.sh &
fi

enable=$(xmllint --xpath "string(//opencv_storage/left_slam)" $PARKINGGO_CFG)
if [ $enable -eq 1 ] 
  then
    $PARKINGGO_DIR/bin/left_lsd_slam.sh &
fi

enable=$(xmllint --xpath "string(//opencv_storage/right_slam)" $PARKINGGO_CFG)
if [ $enable -eq 1 ] 
  then
    $PARKINGGO_DIR/bin/right_lsd_slam.sh &
fi

cd $BASE_DIR/qt-gui
./video &

