#!/bin/sh


sudo docker build --no-cache -t yakircogniteam/bgu_social_navigation:v1 .
sudo docker push yakircogniteam/bgu_social_navigation:v1

#to run the docker:
#xhost +local:docker
#sudo docker run --rm -it --privileged -e ROS_DOMAIN_ID=55 --mount type=bind,source=/home/yakir,target=/home/yakir --net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix yakircogniteam/bgu_social_navigation:v1 ros2 launch social_navigation_launch bringup_launch.py
