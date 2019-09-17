#!/bin/sh
xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$HOME/catkin_ws/src/world/irregular-4.world "&
sleep 5
xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$HOME/catkin_ws/src/world/irregular-4.yaml" &
sleep 5
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch " 