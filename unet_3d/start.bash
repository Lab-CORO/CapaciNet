#!/bin/bash

xhost +local:docker

docker run --name capacitynet2 -it --gpus all --network=host \
  --env DISPLAY=$DISPLAY  \
  --volume  ./ros2_ws/src/capacitynet/:/home/ros2_ws/src/capacitynet/ \
  capacitynet_aarch:latest
