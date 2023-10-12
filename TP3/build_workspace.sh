#!/bin/bash
# Este archivo corre la calibracion en un docker

xhost +local:root &&\
docker run --rm -it --net=host --privileged --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="`pwd`:/home/duser:rw" ros2:humble_with_gazebo bash -c "source /opt/ros/humble/setup.bash; source ~/dev_ws/install/setup.bash; cd ~/dev_ws; export PYTHONWARNINGS='ignore:::setuptools.command.install,ignore:::setuptools.command.easy_install,ignore:::pkg_resources'; colcon build" &&\

# Detiene el contenedor
xhost -local:root
