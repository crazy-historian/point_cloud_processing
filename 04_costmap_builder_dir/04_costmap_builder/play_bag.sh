#!/bin/bash
cd "$(dirname "$0")"
docker run --rm -it -v `pwd`/data:/data osrf/ros:galactic-desktop ros2 bag play -l /data/loc_2021-11-30-19-13-04 --clock
