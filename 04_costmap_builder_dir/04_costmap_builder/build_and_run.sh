#!/bin/bash
cd "$(dirname "$0")"

xhost +

docker build -t mcv_mai/costmap_builder -f ./Dockerfile ./src && \
docker run --rm -it \
    --privileged \
    -e QT_X11_NO_MITSHM=1 \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v `pwd`/src:/src \
    mcv_mai/costmap_builder:latest \
    bash -c "cd /src && \
    colcon build && \
    source /src/install/setup.bash && \
    ros2 launch costmap_builder costmap_builder.launch.py"
