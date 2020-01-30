#!/bin/bash

SCRIPT_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export PYTHONPATH=$PYTHONPATH:`readlink -f $SCRIPT_PATH/../python/dnn_segmentation/semantic_segmentation/`
export PYTHONPATH=$PYTHONPATH:`readlink -f $SCRIPT_PATH/../python/dnn_vehicle/part_car/`
export PYTHONPATH=$PYTHONPATH:`readlink -f $SCRIPT_PATH/../python/dnn_lot/east`
