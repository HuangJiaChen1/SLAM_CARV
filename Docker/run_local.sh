#!/bin/bash

docker run -d -p 5900:5900 --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v /home/ubuntu2404/SLAM_CARV/src:/root/orbslam3/SLAM_CARV/src -v /home/ubuntu2404/SLAM_CARV/include:/root/orbslam3/SLAM_CARV/include --privileged --device=/dev/video0:/dev/video0 --env QT_X11_NO_MITSHM=1 -it islamaali/slam_carv-docker:v1.0
