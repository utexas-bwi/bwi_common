/**
 * \file  bwi_logical_translator.h
 * \brief  Converts high-level logical actions into low-level commands. Maps 
 *         low-level sensor readings into high-level observation fluents.
 *
 * \author  Piyush Khandelwal (piyushk@cs.utexas.edu)
 *
 * Copyright (c) 2013, UT Austin

 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of the <organization> nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 *
 * $ Id: 05/06/2013 11:24:01 AM piyushk $
 *
 **/

#ifndef BWI_LOGICAL_TRANSLATOR_G54K6X7H
#define BWI_LOGICAL_TRANSLATOR_G54K6X7H

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>

#include <bwi_mapper/path_finder.h>
#include <bwi_planning_common/structures.h>
#include <bwi_planning_common/utils.h>
#include <bwi_tools/point.h>
#include <nav_msgs/GetPlan.h>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace bwi_logical_translator {

  class BwiLogicalTranslator {

    public:

      BwiLogicalTranslator();

      bool initialize();

      bool isDoorOpen(const std::string &door_name);

      bool getApproachPoint(const std::string &door_name,
                            const bwi::Point2f &current_location,
                            bwi::Point2f &point, float &yaw);

      bool getThroughDoorPoint(const std::string &door_name,
                               const bwi::Point2f &current_location,
                               bwi::Point2f &point, float &yaw);

      bool isRobotFacingDoor(
              const bwi::Point2f &current_location,
              float yaw, float threshold, const std::string &door_name);

      bool getRobotFacingDoor(
              const bwi::Point2f &current_location,
              float yaw, float threshold, std::string &door_name);

      bool isRobotBesideDoor(
              const bwi::Point2f &current_location,
              float yaw, float threshold, const std::string &door_name);

      bool isRobotFacingLocation(
          const bwi::Point2f &current_location,
          float yaw, float threshold, const std::string &object_name);

      bool isRobotBesideObject(
          const bwi::Point2f& current_location,
          float yaw, float threshold, std::string& object_name);

      bool isObjectApproachable(const std::string& object_name, 
          const bwi::Point2f& current_location);

      inline bool getObjectApproachLocation(const std::string& object_name,
          geometry_msgs::Pose& pose) {
        if (location_approach_map_.find(object_name) ==
            location_approach_map_.end()) {
          return false; 
        }
        pose = location_approach_map_[object_name];
        return true;
      }

      bool getNearbyLocations(const bwi::Point2f &near_to, float threshold,
                              std::vector<std::string> &nearby_location_names);

      size_t getRegionIdx(
          const bwi::Point2f& region_pt);

      inline size_t getRegionIdx(
          const std::string& region_str) const {
        for (size_t i = 0; i < regions_.size(); ++i) {
          if (regions_[i] == region_str) {
            return i;
          }
        }
        return (size_t)-1;
      }

      inline size_t getDoorIdx(const std::string& door_str) const {
        for (size_t i = 0; i < doors_.size(); ++i) {
          if (doors_[i].name == door_str) {
            return i;
          }
        }
        return (size_t)-1;
      }

      inline std::string getRegionString(size_t idx) const {
        if (idx >= regions_.size()) {
          ROS_WARN_STREAM("Queried region name for non-existent index " << idx);
          return "";
        }
        return regions_[idx];
      }

      inline std::string getDoorString(size_t idx) const {
        if (idx >= doors_.size()) {
          ROS_WARN_STREAM("Queried door name for non-existent index " << idx);
          return "";
        }
        return doors_[idx].name;
      }

      inline size_t getNumDoors() const {
        return doors_.size();
      }

      bool is_door(const std::string& door_name) const {
          return name_to_door.find(door_name) != name_to_door.end();
      }

    protected:

      std::string global_frame_id_;

      std::vector<bwi_planning_common::Door> doors_;
      std::map<std::string, bwi_planning_common::Door> name_to_door;
      std::map<std::string, boost::shared_ptr<bwi_mapper::PathFinder> > door_approachable_space_1_;
      std::map<std::string, boost::shared_ptr<bwi_mapper::PathFinder> > door_approachable_space_2_;

      std::vector<std::string> regions_;
      std::vector<int32_t> region_map_;
      std::map<std::string, geometry_msgs::Pose> location_approach_map_;

      pcl::PointCloud<pcl::PointXY>::Ptr location_points;
      pcl::KdTreeFLANN<pcl::PointXY> location_tree;
      std::map<int, std::string> index_to_name;

      std::map<std::string, boost::shared_ptr<bwi_mapper::PathFinder> > location_approachable_space_;

      nav_msgs::OccupancyGrid map_;
      nav_msgs::OccupancyGrid map_with_doors_;
      nav_msgs::OccupancyGrid inflated_map_with_doors_;
      nav_msgs::MapMetaData info_;

      boost::shared_ptr<ros::NodeHandle> nh_;
      ros::ServiceClient make_plan_client_;
      ros::ServiceClient static_costmap_toggle_client_;
      bool make_plan_client_initialized_;

      bool static_costmap_toggle_client_initialized_;
      void initializeStaticCostmapToggleService();
      void enableStaticCostmap(bool value);

      bool initialized_;

  }; /* BwiLogicalTranslator */
  
} /* bwi_logical_translator */

#endif /* end of include guard: BWI_LOGICAL_TRANSLATOR_G54K6X7H */
