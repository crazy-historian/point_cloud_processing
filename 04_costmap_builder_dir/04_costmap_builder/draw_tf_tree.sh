#!/bin/bash
cd "$(dirname "$0")"
docker run --rm -it -v `pwd`:/data -w /data osrf/ros:galactic-desktop ros2 run tf2_tools view_frames
