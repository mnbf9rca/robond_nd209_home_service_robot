# robond_nd209_home_service_robot
Final project for Udacity's robotic nanodegree project

# notes
 - only works with python 2.7 or you get error `ImportError: dynamic module does not define module export function (PyInit__tf2)` https://github.com/ros/geometry2/issues/411
 - remember to run `rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y`
 - additional dependency `pip install catkin_pkg pyyaml empy rospkg numpy defusedxml`
 - save using `rosrun map_server map_saver`