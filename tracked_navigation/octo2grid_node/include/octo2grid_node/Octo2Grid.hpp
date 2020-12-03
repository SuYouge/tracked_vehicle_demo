/*
 * OctomapToGridmap.hpp
 *
 *  Created on: May 03, 2017
 *      Author: Jeff Delmerico
 *   Institute: University of Zürich, Robotics and Perception Group
 */

#pragma once

// ROS
#include <ros/ros.h>

#include <grid_map_ros/grid_map_ros.hpp>

#include <string>

namespace octo2grid {

/*!
 * Receives a volumetric OctoMap and converts it to a grid map with an elevation layer.
 * The grid map is published and can be viewed in Rviz.
 */
class OctomapToGridmap
{
 public:

  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  OctomapToGridmap(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~OctomapToGridmap();

  /*!
  * Reads and verifies the ROS parameters.
  * @return true if successful.
  */
  bool readParameters();

  void convertAndPublishMap();

 private:

  //! ROS nodehandle.
  ros::NodeHandle& nodeHandle_;

  //! Grid map publisher.
  ros::Publisher gridMapPublisher_;

  //! Octomap publisher.
  // ros::Publisher octomapPublisher_;

  //! Grid map data.
  grid_map::GridMap map_;

  //! Name of the grid map topic.
  std::string octomapServiceTopic_;

  //! Octomap service client
  ros::ServiceClient client_;

  //! Bounding box of octomap to convert.
  float minX_;
  float maxX_;
  float minY_;
  float maxY_;
  float minZ_;
  float maxZ_;
};

} /* namespace */
