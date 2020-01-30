#!/bin/bash

SCRIPT=$( readlink -m $( type -p $0 ))
BASE_DIR=`dirname ${SCRIPT}`

if [ $# -eq 0 ]
then
  echo "Usage: `basename $0` [Optional: t]"
  echo "Set Lot detection model. Optional argument t to set model for tracking."
  echo -e "\n"
  TARGET_MODEL=$BASE_DIR/../python/dnn_lot/east/model
else
  TARGET_MODEL=$BASE_DIR/../python/dnn_lot/east/model_tracking
fi

MODELS_DIR=~/catkin_ws/dnn_lot_model

cd $MODELS_DIR

current_model=$( readlink -m $TARGET_MODEL )
echo $current_model

echo -e "\nAvailable models (* current model):\n"

counter=0
for dir in `ls -d */ | sed -e "s/\///g"`; do
  models[$counter]=$MODELS_DIR/$dir

  if [ "${models[counter]}" = "$current_model" ];
  then
    echo -e "\t*$counter: $dir"
  else
    echo -e "\t $counter: $dir"
  fi

  counter=$((counter+1))
done

echo -e "\n"

index=-1
while [[ "$index" -lt 0 || "$index" -gt $((counter-1)) ]]; do
  read -p "Enter model to set one: [0 - $((counter-1))]" index
done

rm $TARGET_MODEL &> /dev/null
ln -s ${models[index]} $TARGET_MODEL





