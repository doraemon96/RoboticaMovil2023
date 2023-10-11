#!/bin/bash
# Este archivo corre la calibracion en un docker

xhost +local:root &&\
docker run --rm -itd --net=host --privileged --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="`pwd`:/home/duser:rw" ros2:humble_with_gazebo &&\

## CORRE EL ROSBAG DE CALIBRACION
gnome-terminal -- sh -c 'docker exec -it `docker ps | awk '\''/ros2:humble_with_gazebo/ { print $1 }'\''` bash -c "source /opt/ros/humble/setup.bash; source ~/dev_ws/install/setup.bash; ros2 bag play --disable-keyboard-controls ./EuRoC/cam_checkerboard/cam_checkerboard_rosbag2"' &&

## LANZA EL CALIBRADOR EN UNA TERMINAL Y ESPERA QUE TERMINE
gnome-terminal --wait -- sh -c 'docker exec -it `docker ps | awk '\''/ros2:humble_with_gazebo/ { print $1 }'\''` bash -c "source /opt/ros/humble/setup.bash; source ~/dev_ws/install/setup.bash; ros2 run camera_calibration cameracalibrator --approximate 0.1 --size 7x6 --square 0.108 --ros-args -r right:=/cam0 -r left:=/cam1 -r right_camera:=/cam0 -r left_camera:=/cam1"' &&

## COPIA LOS ARCHIVOS DE CALIBRACION
gnome-terminal -- sh -c 'docker exec -it `docker ps | awk '\''/ros2:humble_with_gazebo/ { print $1 }'\''` bash -c "cd EuRoC/cam_checkerboard &&\
mv /tmp/calibrationdata.tar.gz . &&\
tar -xf calibrationdata.tar.gz right.yaml left.yaml"' &&

# Detiene el contenedor
docker stop `docker ps | awk '/ros2:humble_with_gazebo/ { print $1 }'`
xhost -local:root
