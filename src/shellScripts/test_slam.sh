#!/bin/sh
xterm -e "roslaunch rob_bot world.launch world_file:=~/catkin_ws/src/rob_bot/worlds/kitchen_dining.world 2>/dev/null &"
sleep 5