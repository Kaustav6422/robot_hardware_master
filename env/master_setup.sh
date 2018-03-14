#!/usr/bin/env bash

export master=192.168.0.4
export robot2=192.168.0.22

echo master
echo robot2

export ROS_IP=192.168.0.4
export ROS_HOSTNAME="master"
export ROS_MASTER_URI=http://192.168.0.4:11311
export ROSLAUNCH_SSH_UNKNOWN=1
#export ROS_ROOT=/opt/ros/kinetic/share/ros
#export ROS_PACKAGE_PATH=/home/kaustav/catkin_ws/src:/opt/ros/kinetic/share 

source ~/.bashrc
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
exec "$@"
