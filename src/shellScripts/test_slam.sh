#!/bin/sh
echo "launching world"

xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$HOME/catkin_ws/src/world/u-2.world "&
sleep 5
echo "launching turtlebot"
xterm -e " roslaunch turtlebot_gazebo gmapping_demo.launch" &
sleep 5
echo "launching rviz"
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 5
echo "launching teleop"
xterm -e " roslaunch turtlebot_teleop keyboard_teleop.launch "