/*
* modified from https://raw.githubusercontent.com/ros-visualization/visualization_tutorials/indigo-devel/visualization_marker_tutorials/src/basic_shapes.cpp
*
*/

/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <std_msgs/String.h>

#include <string>

std::string goal_state = "no_goal"; // no_goal, to_pick_up, at_goal, to_drop_off
void goalstateCallback(const std_msgs::String::ConstPtr &msg)
{
  ::goal_state = msg->data.c_str();
  ROS_INFO("goal_state message received: '[%s]'", ::goal_state.c_str());
}

// %Tag(INIT)%
int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate rate(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  // subscribe to goal_state to see whether we're moving or at goal
  // beware magic strings :)
  // can send no_goal, to_pick_up, at_goal, to_drop_off
  ros::Subscriber goal_state_sub = n.subscribe("/goal_state", 1000, goalstateCallback);

  while (ros::ok())
  {
    ROS_INFO("goal_state is: '[%s]'", ::goal_state.c_str());
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    // %EndTag(MARKER_INIT)%

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    // %Tag(NS_ID)%
    marker.ns = "basic_shapes";
    marker.id = 0;
    // %EndTag(NS_ID)%

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::CUBE;
    ;

    marker.pose.orientation.w = 0.73;

    marker.pose.position.z = 0.5;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();

    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    if (goal_state == "to_pick_up")
    {

      marker.pose.position.x = 3.2;
      marker.pose.position.y = 4.3;
      marker.action = visualization_msgs::Marker::ADD;

      marker_pub.publish(marker);
      ROS_INFO("Published pick up marker");
    }
    else if (goal_state == "to_drop_off")
    {
      marker.pose.position.x = -3.0;
      marker.pose.position.y = 2.4;
      marker.action = visualization_msgs::Marker::ADD;

      marker_pub.publish(marker);
      ROS_INFO("Published drop off marker");
    }
    else // anything else really - at_goal or no_goal
    {
      // delete marker
      marker.action = visualization_msgs::Marker::DELETE;
      marker_pub.publish(marker);
      ROS_INFO("Deleted marker");
    }

    //r.sleep();
    ros::spinOnce();
  }
}
