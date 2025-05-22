#!/bin/bash

xhost +local:docker

docker run --name capacitynet2 -it --gpus all --network=host \
  --env DISPLAY=$DISPLAY  \
  --volume  $PWD/:/workspace/capacitynet/ \
  pytorch/pytorch