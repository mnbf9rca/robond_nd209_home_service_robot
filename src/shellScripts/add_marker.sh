#!/bin/sh
xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$PWD/catkin_ws/src/world/irregular-4.world "&
sleep 5
xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$PWD/catkin_ws/src/world/irregular-4.yaml" &
sleep 5
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 5
xterm -e " rosrun add_markers add_markers_node " 