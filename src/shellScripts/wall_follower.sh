#!/bin/sh
xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$HOME/catkin_ws/src/world/u.world "&
sleep 5
xterm -e " roslaunch turtlebot_gazebo gmapping_demo.launch custom_gmapping_launch_file:=$HOME/catkin_ws/src/turtlebot_simulator/turtlebot_gazebo/launch/gmapping_optimised.launch.xml " &
sleep 5
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 5
xterm -e " rosrun wall_follower wall_follower_node "
