#!/bin/bash
cd "$(dirname "$0")"
docker run --rm -it -e QT_X11_NO_MITSHM=1 -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v `pwd`/src/costmap_builder/rviz/default.rviz:/default.rviz osrf/ros:galactic-desktop rviz2 -d /default.rviz
