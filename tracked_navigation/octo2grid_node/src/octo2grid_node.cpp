/*
 * octomap_to_gridmap_demo_node.cpp
 *
 *  Created on: May 03, 2017
 *      Author: Jeff Delmerico
 *   Institute: University of Zürich, Robotics and Perception Group
 */

#include <ros/ros.h>
#include "octo2grid_node/Octo2Grid.hpp"

int main(int argc, char** argv)
{
  // Initialize node and publisher.
  ros::init(argc, argv, "octomap_to_gridmap_demo");
  ros::NodeHandle nh("~");
  octo2grid::OctomapToGridmap OctomapToGridmap(nh);
  ROS_ERROR("Init node");
  ros::Duration(2.0).sleep();

  ros::Rate r(0.1); // 1 hz
  while (ros::ok())
  {
    // octo2grid::OctomapToGridmap OctomapToGridmap(nh);
    OctomapToGridmap.convertAndPublishMap();
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
