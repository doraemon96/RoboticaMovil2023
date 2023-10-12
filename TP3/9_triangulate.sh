#!/bin/bash
# Este archivo corre la calibracion en un docker

xhost +local:root &&\
docker run --rm -itd --net=host --privileged --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="`pwd`:/home/duser:rw" ros2:humble_with_gazebo &&\


## LANZA RVIZ
gnome-terminal -- sh -c 'docker exec -it `docker ps | awk '\''/ros2:humble_with_gazebo/ { print $1 }'\''` bash -c "source /opt/ros/humble/setup.bash; rviz2 -d config.rviz"'


## LAUNCH ROS CON TODOS LOS NODOS, PUBLICANDO IMAGENES EN RVIZ
gnome-terminal --wait -- sh -c 'docker exec -it `docker ps | awk '\''/ros2:humble_with_gazebo/ { print $1 }'\''` bash -c "source /opt/ros/humble/setup.bash; cd dev_ws; source install/setup.bash; ros2 launch euroc_stereo2 euroc_ds_tp3.launch.py draw_matches:='\''false'\'' rosbag_path:='\''../EuRoC/V1_01_easy'\''"'



# Detiene el contenedor
docker stop `docker ps | awk '/ros2:humble_with_gazebo/ { print $1 }'`
xhost -local:root
