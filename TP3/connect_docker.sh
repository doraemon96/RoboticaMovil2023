#/usr/bin/bash

docker exec -it `docker ps | awk '/ros2:humble_with_gazebo/ { print $1 }'` bash
