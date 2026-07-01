#!/bin/bash

xhost +local:docker

docker run --name capacitynet2 -it --runtime nvidia -e NVIDIA_VISIBLE_DEVICES=all \
  --network=host \
  --env DISPLAY=$DISPLAY \
  --env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  --volume  ./ros2_ws/src/capacitynet/:/home/ros2_ws/src/capacitynet/ \
  capacitynet:aarch64-jazzy
