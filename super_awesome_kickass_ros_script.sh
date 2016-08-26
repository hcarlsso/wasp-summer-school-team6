#!/bin/bash

SRC=/home/wasp/catkin_ws/
DRONE_WIFI="WaspGroup6"

echo "Sourcing stuff"
source "$SRC/devel/setup.bash"

#
# Set wifi on the ardrone
#
echo "Connecting to the drone Wifi"
nmcli c up id $DRONE_WIFI
echo "Waiting for the wifi to connect"
#if ["iwgetid wlan0 -r" == $DRONE_WIFI];
#    then break;
#    else echo "Wrong wifi !"; exit;
#fi
echo "Changing the drone Wifi"
echo "./data/wifi.sh" | telnet 192.168.1.1
echo "Changing the drone Wifi DONE"

#
# Switch to the control room Network
#
echo "Please connect the control room cable"
nmcli c up id $TEAM_WIFI
read -p "Press any key when connected to continue... " -n1 -s

#
# Roscore
#
echo "Start roscore !"
roslaunch launch/all.launch

# All the magic happens here

pkill roscore &> "$SRC/kills.log"

#
# Go back to eduroam
#
echo "Revert to proper eduroam"
nmcli c up id eduroam

echo "Bye bye !"
