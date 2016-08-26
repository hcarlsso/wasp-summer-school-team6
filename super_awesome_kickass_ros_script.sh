#!/bin/bash
SRC=/home/wasp/catkin_ws/

echo "Sourcing stuff"
source "$SRC/devel/setup.bash"

#
# Roscore
#
echo "Start roscore !"
roslaunch launch/all.launch

pkill roscore &> "$SRC/kills.log"
echo "Bye bye !"
