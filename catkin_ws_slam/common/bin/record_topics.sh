#!/bin/bash

SCRIPT=$( readlink -m $( type -p $0 ))
BASE_DIR=`dirname ${SCRIPT}`

RECORD_DIR=$BASE_DIR/../records

mkdir $RECORD_DIR

cd $RECORD_DIR

rosbag record mcu_msg dnn_lots front_image back_image left_image right_image

