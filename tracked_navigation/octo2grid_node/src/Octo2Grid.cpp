/*
 * OctomapToGridmap.cpp
 *
 *  Created on: May 03, 2017
 *      Author: Jeff Delmerico
 *   Institute: University of Zürich, Robotics and Perception Group
 */

#include "octo2grid_node/Octo2Grid.hpp"

#include <octo2grid_core/GridMapOctomapConverter.hpp>
// #include <grid_map_octomap/GridMapOctomapConverter.hpp>

#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomap.h>

#include <rtabmap/core/OctoMap.h>

namespace octo2grid
{

  OctomapToGridmap::OctomapToGridmap(ros::NodeHandle &nodeHandle)
      : nodeHandle_(nodeHandle),
        map_(grid_map::GridMap({"elevation"}))
  {
    ROS_ERROR("Reading param");
    readParameters();
    client_ = nodeHandle_.serviceClient<octomap_msgs::GetOctomap>(octomapServiceTopic_);
    map_.setBasicLayers({"elevation"});
    gridMapPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
    // octomapPublisher_ = nodeHandle_.advertise<octomap_msgs::Octomap>("octomap", 1, true);
  }

  OctomapToGridmap::~OctomapToGridmap()
  {
  }

  bool OctomapToGridmap::readParameters()
  {
    nodeHandle_.param("octomap_service_topic", octomapServiceTopic_, std::string("/rtabmap/octomap_full"));
    nodeHandle_.param("min_x", minX_, NAN);
    nodeHandle_.param("max_x", maxX_, NAN);
    nodeHandle_.param("min_y", minY_, NAN);
    nodeHandle_.param("max_y", maxY_, NAN);
    nodeHandle_.param("min_z", minZ_, NAN);
    nodeHandle_.param("max_z", maxZ_, NAN);
    return true;
  }

  void OctomapToGridmap::convertAndPublishMap()
  {
    octomap_msgs::GetOctomap srv;
    if (!client_.call(srv))
    {
      ROS_ERROR_STREAM("Failed to call service: " << octomapServiceTopic_);
      return;
    }
    else
    {
      ROS_ERROR("Call service successfully");
    }

    // creating octree ColorOcTree*
    // octomap::OcTree* octomap = nullptr;
    octomap::ColorOcTree* octomap = nullptr;
    // octomap::OcTree* octomap = new octomap::OcTree(0.1);
    
    // ROS_INFO("Octomap resolution: [%f]", (float)srv.response.map.resolution);
    // ROS_INFO("Octomap binary: [%d]", (bool)srv.response.map.binary);
    // ROS_INFO("Octomap data size: [%d]", srv.response.map.data.size());
    // ROS_INFO("Octomap id: [%s]", srv.response.map.id.c_str());
    // ROS_INFO("Octomap frame_id: [%s]", srv.response.map.header.frame_id.c_str());
    // octomap::OcTree* octomap = nullptr;
    octomap::AbstractOcTree *tree = octomap_msgs::msgToMap(srv.response.map);

    // octomap::AbstractOcTree* tree;
    // if (srv.response.map.id.c_str() == "ColorOcTree"){
    //   octomap::ColorOcTree* color_octomap = new octomap::ColorOcTree(srv.response.map.resolution);    
    //   readTree(color_octomap, srv.response.map);
    //   tree = color_octomap;
    // }


    if (tree)
    {
      octomap = dynamic_cast<octomap::ColorOcTree* >(tree);
      if (octomap)
      {
        grid_map::Position3 min_bound;
        grid_map::Position3 max_bound;
        octomap->getMetricMin(min_bound(0), min_bound(1), min_bound(2));
        octomap->getMetricMax(max_bound(0), max_bound(1), max_bound(2));
        ROS_ERROR_STREAM("get TreeType = "<<octomap->getTreeType(););
        ROS_ERROR("set max min !");
        if (!std::isnan(minX_))
          min_bound(0) = minX_;
        if (!std::isnan(maxX_))
          max_bound(0) = maxX_;
        if (!std::isnan(minY_))
          min_bound(1) = minY_;
        if (!std::isnan(maxY_))
          max_bound(1) = maxY_;
        if (!std::isnan(minZ_))
          min_bound(2) = minZ_;
        if (!std::isnan(maxZ_))
          max_bound(2) = maxZ_;
        bool res = grid_map::GridMapOctomapConverter::fromOctomap(*octomap, "elevation", map_, &min_bound, &max_bound);
        if (!res)
        {
          ROS_ERROR("Failed to call convert Octomap.");
          return;
        }
        else
        {
          ROS_ERROR("Successfully to call convert Octomap.");
        }
        // map_.setFrameId("/world");
        map_.setFrameId(srv.response.map.header.frame_id);
        // Publish as grid map.
        grid_map_msgs::GridMap gridMapMessage;
        grid_map::GridMapRosConverter::toMessage(map_, gridMapMessage);
        gridMapPublisher_.publish(gridMapMessage);
        ROS_ERROR("Call convert Octomap successfully");
      }else{
        ROS_ERROR("Failed to call convert Octomap.");
      }
      delete octomap;
    }
    else
    {
      ROS_ERROR("Convert break");
      return;
    }

    // Also publish as an octomap msg for visualization
    // octomap_msgs::Octomap octomapMessage;
    // octomap_msgs::fullMapToMsg(*octomap, octomapMessage);
    // octomapMessage.header.frame_id = map_.getFrameId();
    // octomapPublisher_.publish(octomapMessage);
  }

} // namespace octo2grid
