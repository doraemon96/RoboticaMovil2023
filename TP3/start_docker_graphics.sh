#!/usr/bin/bash

xhost +local:root &&\
docker run --rm -it --net=host --privileged --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="`pwd`:/home/duser:rw" ros2:humble_with_gazebo &&\
xhost -local:root 
