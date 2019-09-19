#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/String.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char **argv)
{
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while (!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // publish whether at goal or not
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher goal_state_pub = n.advertise<std_msgs::String>("goal_state", 1000);
  // create msg to hold message
  std_msgs::String msg;
  std::stringstream ss_no_goal;

  msg.data = "no_goal";
  goal_state_pub.publish(msg);

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  while (goal_state_pub.getNumSubscribers() < 1)
  {
    if (!ros::ok())
    {
      return 0;
    }
    ROS_WARN_ONCE("cannot navigate until goal_state has subscribers");
    sleep(1);
  }

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = 3.2;
  goal.target_pose.pose.position.y = 4.3;
  goal.target_pose.pose.orientation.w = 0.73;

  // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal to pick up");
  ac.sendGoal(goal);
  msg.data = "to_pick_up";
  goal_state_pub.publish(msg);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Hooray, reached pick up target");
    msg.data = "at_goal";
    goal_state_pub.publish(msg);
    ros::Duration(5.0).sleep();
    // set new target
    goal.target_pose.pose.position.x = -3.0;
    goal.target_pose.pose.position.y = 2.4;
    goal.target_pose.pose.orientation.w = -0.30;
    ROS_INFO("Sending goal to drop off");
    ac.sendGoal(goal);
    msg.data = "to_drop_off";
    goal_state_pub.publish(msg);

    // Wait an infinite time for the results
    ac.waitForResult();
    // Check if the robot reached its goal
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Hooray, reached drop off location");
      msg.data = "at_goal";
      goal_state_pub.publish(msg);
      ros::Duration(5.0).sleep();
    }
    else
      ROS_INFO("The base failed to reach drop off");
  }
  else
    ROS_INFO("The base failed to move for some reason");

  return 0;
}