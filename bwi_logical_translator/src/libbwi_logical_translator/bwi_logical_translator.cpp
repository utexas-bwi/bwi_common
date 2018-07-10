/**
 * \file  bwi_logical_translator.cpp
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
 * $ Id: 12/14/2013 04:35:22 PM piyushk $
 *
 **/

#include <dynamic_reconfigure/Reconfigure.h>
#include <tf/transform_datatypes.h>

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/join.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/thread/thread.hpp>
#include <bwi_mapper/map_inflator.h>
#include <bwi_mapper/map_loader.h>
#include <bwi_mapper/map_utils.h>
#include <bwi_mapper/point_utils.h>

#include <bwi_logical_translator/bwi_logical_translator.h>
#include <pcl/point_cloud.h>
#include <cmath>
#include <pcl/impl/point_types.hpp>

#define SCALAR 1.1

using std::vector;

namespace bwi_logical_translator {

  BwiLogicalTranslator::BwiLogicalTranslator() :
      make_plan_client_initialized_(false), 
      static_costmap_toggle_client_initialized_(false), 
      initialized_(false) {

    nh_.reset(new ros::NodeHandle);
    ros::param::param<std::string>("~global_frame_id", global_frame_id_, "level_mux_map");
  }

  bool BwiLogicalTranslator::initialize() {
    ROS_INFO_STREAM("BwiLogicalTranslator: Initializing...");

    std::string map_file, data_directory;
    std::vector<std::string> required_parameters;
    if (!ros::param::get("~map_file", map_file)) {
      required_parameters.emplace_back("~map_file");
    }
    if (!ros::param::get("~data_directory", data_directory)) {
      required_parameters.emplace_back("~data_directory");
    }
    if (!required_parameters.empty()) {
      std::string message = "BwiLogicalTranslator: Required parameters [" +
        boost::algorithm::join(required_parameters, ", ") + "] missing!";
      ROS_FATAL_STREAM(message);
      throw std::runtime_error(message);
    }

    std::string door_file = bwi_planning_common::getDoorsFileLocationFromDataDirectory(data_directory);
    ROS_INFO_STREAM("BwiLogicalTranslator: Reading door file: " + door_file);
    bwi_planning_common::readDoorFile(door_file, doors_);
    for (const auto& door: doors_) {
        name_to_door.insert({door.name, door});
    }

    std::string region_file = bwi_planning_common::getLocationsFileLocationFromDataDirectory(data_directory);
    ROS_INFO_STREAM("BwiLogicalTranslator: Reading regions file: " + region_file);
    bwi_planning_common::readLocationFile(region_file, regions_, region_map_);

    std::string location_file = bwi_planning_common::getObjectsFileLocationFromDataDirectory(data_directory);
    ROS_INFO_STREAM("BwiLogicalTranslator: Checking if locations file exists: " + location_file);
    if (boost::filesystem::exists(location_file)) {
      ROS_INFO_STREAM("BwiLogicalTranslator:   Locations file exists. Reading now!");
      bwi_planning_common::readObjectApproachFile(location_file, location_approach_map_);
    }
    int i = 0;
    location_points = pcl::PointCloud<pcl::PointXY>::Ptr(new pcl::PointCloud<pcl::PointXY>);
    for (const auto& door: doors_) {
      pcl::PointXY point = {door.door_center.x, door.door_center.y};
      location_points->push_back(point);
      index_to_name.insert({i, door.name});
      i++;
    }

    for (const auto& location: location_approach_map_) {
      auto pose = location.second;
      pcl::PointXY point = {pose.position.x, pose.position.y};
      location_points->push_back(point);
      index_to_name.insert({i, location.first});
      i++;
    }
    location_tree.setInputCloud(location_points);

    bwi_mapper::MapLoader mapper(map_file);
    mapper.getMap(map_);
    map_.header.stamp = ros::Time::now();
    map_.header.frame_id = global_frame_id_;
    info_ = map_.info;

    std::string map_with_doors_file = bwi_planning_common::getDoorsMapLocationFromDataDirectory(data_directory);
    mapper = bwi_mapper::MapLoader(map_with_doors_file);
    mapper.getMap(map_with_doors_);
    map_with_doors_.header.stamp = ros::Time::now();
    map_with_doors_.header.frame_id = global_frame_id_;

    // Inflating map by 20cm should get rid of any tiny paths to the goal.
    bwi_mapper::inflateMap(0.2, map_with_doors_, inflated_map_with_doors_);

    // We'll do lazy initialization of the approachable space. Just clear it for now, and we'll populate it as
    // necessary.
    door_approachable_space_1_.clear();
    door_approachable_space_2_.clear();
    location_approachable_space_.clear();

    initialized_ = true;
    return true;
  }

  bool BwiLogicalTranslator::isDoorOpen(const std::string &door_name) {

    if (!initialized_) {
      ROS_ERROR_STREAM("BwiLogicalTranslator : requesting data without being initialized!");
      return false;
    }

    if (name_to_door.find(door_name) == name_to_door.end()) {
        ROS_ERROR_STREAM("Attempted to query openess of non-existent door: " << door_name);
        assert (false);
        return false;
    }

    enableStaticCostmap(false);
    // TODO: this should not be necesary, since we make service calls 
    boost::this_thread::sleep(boost::posix_time::milliseconds(250));

    bwi_mapper::Point2f start_pt, goal_pt;
    float start_yaw, goal_yaw;

    auto door = name_to_door[door_name];
    start_pt = door.approach_points[0];
    goal_pt = door.approach_points[1];
    start_yaw = door.approach_yaw[0];
    goal_yaw = door.approach_yaw[1];

    nav_msgs::GetPlan srv;
    geometry_msgs::PoseStamped &start = srv.request.start;
    geometry_msgs::PoseStamped &goal = srv.request.goal;
    start.header.frame_id = goal.header.frame_id = global_frame_id_;
    start.header.stamp = goal.header.stamp = ros::Time::now();

    start.pose.position.x = start_pt.x;
    start.pose.position.y = start_pt.y;
    start.pose.position.z = 0;
    start.pose.orientation = tf::createQuaternionMsgFromYaw(start_yaw);

    goal.pose.position.x = goal_pt.x;
    goal.pose.position.y = goal_pt.y;
    goal.pose.position.z = 0;
    goal.pose.orientation = tf::createQuaternionMsgFromYaw(goal_yaw);
    srv.request.tolerance = -1.0f;//0.5 + 1e-6;

    float min_distance = cv::norm(start_pt - goal_pt);

    int counter = 0;

    if (!make_plan_client_initialized_) {
      ROS_INFO_STREAM("BwiLogicalTranslator: Waiting for make_plan service..");
      make_plan_client_ = nh_->serviceClient<nav_msgs::GetPlan>("move_base/NavfnROS/make_plan");
      make_plan_client_.waitForExistence();
      ROS_INFO_STREAM("BwiLogicalTranslator: make_plan service found!");
      make_plan_client_initialized_ = true;
    }

    // Check three times if this door is open. Sometimes, the costmap is out of sync or something, and the global
    // planner returns paths through obstacles. This bug is true for both navfn and global_planner.
    for (int i = 0; i < 3; i++) {
      if (make_plan_client_.call(srv)) {
        if (!srv.response.plan.poses.empty()) {
          // Valid plan received. Check if plan distance seems reasonable
          float distance = 0;
          geometry_msgs::Point old_pt =
            srv.response.plan.poses[0].pose.position;
          for (size_t j = 1; j < srv.response.plan.poses.size(); ++j) {
            geometry_msgs::Point current_pt =
              srv.response.plan.poses[j].pose.position;
            distance += sqrt(pow(current_pt.x - old_pt.x, 2) +
                             pow(current_pt.y - old_pt.y, 2));
            old_pt = current_pt;
          }
					ROS_INFO_STREAM("BwiLogicalTranslator: plan distance to go through door is: " << distance);
					ROS_INFO_STREAM("Comparing this plan to distance threshhold of " << SCALAR * min_distance);
          if (distance <  SCALAR *  min_distance) {
            //return true;
            counter++;
						ROS_INFO_STREAM("BwiLogicalTranslator: door open, counter is " << counter);
          } else {
            //return false; // returned path probably through some other door
            ROS_INFO_STREAM("BwiLogicalTranslator: isDoorOpen: Returned path is too long.");
            counter = 0;
          }
        } else {
          //return false; // this is ok. it means the door is closed
          ROS_INFO_STREAM("BwiLogicalTranslator: isDoorOpen: Could not find path, door is likely closed.");
          counter = 0;
        }
      } else {
        //return false; // shouldn't be here. the service has failed
        counter = 0;
      }
    }

    // TODO: set this to whatever state it was in.
    enableStaticCostmap(true);
    // We have to see the door open three consecutive times
      return counter == 3;
  }

  bool BwiLogicalTranslator::getApproachPoint(const std::string &door_name,
                                              const bwi_mapper::Point2f &current_location,
                                              bwi_mapper::Point2f &point, float &yaw) {

    if (!initialized_) {
      ROS_ERROR_STREAM("BwiLogicalTranslator : requesting data without being initialized!");
      return false;
    }

    if (name_to_door.find(door_name) == name_to_door.end()) {
      ROS_ERROR_STREAM("Attempted to get approach point of non-existent door: " << door_name);
      assert (false);
      return false;
    }
    const auto& door = name_to_door[door_name];
    // See if we've calculated the approachable space for this door.
    if (door_approachable_space_1_.find(door_name) == door_approachable_space_1_.end()) {

      const bwi_mapper::Point2d approach_pt_1(bwi_mapper::toGrid(door.approach_points[0], info_));
      door_approachable_space_1_[door_name] = boost::shared_ptr<bwi_mapper::PathFinder>(new bwi_mapper::PathFinder(inflated_map_with_doors_, approach_pt_1));
    }
    if (door_approachable_space_2_.find(door_name) == door_approachable_space_2_.end()) {
      const bwi_mapper::Point2d approach_pt_2(bwi_mapper::toGrid(door.approach_points[1], info_));
      door_approachable_space_2_[door_name] = boost::shared_ptr<bwi_mapper::PathFinder>(new bwi_mapper::PathFinder(inflated_map_with_doors_, approach_pt_2));
    }

    // Find the approach point to which we can find a path. If both approach points can be reached, fine the approach
    // point which is closer.
    bwi_mapper::Point2d grid(bwi_mapper::toGrid(current_location, info_));
    int distance_1 = door_approachable_space_1_[door_name]->getManhattanDistance(grid);
    int distance_2 = door_approachable_space_2_[door_name]->getManhattanDistance(grid);
    if (distance_1 >= 0 || distance_2 >= 0) {
      if (distance_1 >= 0 && (distance_1 < distance_2 || distance_2 < 0)) {
        point = door.approach_points[0];
        yaw = door.approach_yaw[0];
      } else {
        point = door.approach_points[1];
        yaw = door.approach_yaw[1];
      }
      return true;
    }

    /* The door is not approachable from the current location */
    return false;
  }

  bool BwiLogicalTranslator::isObjectApproachable(const std::string& object_name,
                                                     const bwi_mapper::Point2f& current_location) {

    if (!initialized_) {
      ROS_ERROR_STREAM("BwiLogicalTranslator : requesting data without being initialized!");
      return false;
    }

    if (location_approachable_space_.find(object_name) == location_approachable_space_.end()) {
      const geometry_msgs::Pose& object_pose = location_approach_map_[object_name];
      const bwi_mapper::Point2d approach_pt(bwi_mapper::toGrid(bwi_mapper::Point2f(object_pose.position.x,
                                                                                   object_pose.position.y),
                                                               info_));
      location_approachable_space_[object_name] = boost::shared_ptr<bwi_mapper::PathFinder>(new bwi_mapper::PathFinder(inflated_map_with_doors_, approach_pt));
    }

    bwi_mapper::Point2d grid_pt(bwi_mapper::toGrid(current_location, info_));
    return location_approachable_space_[object_name]->pathExists(grid_pt);
  }

  bool BwiLogicalTranslator::getThroughDoorPoint(const std::string &door_name,
                                                 const bwi_mapper::Point2f &current_location,
                                                 bwi_mapper::Point2f &point, float &yaw) {

    if (!initialized_) {
      ROS_ERROR_STREAM("BwiLogicalTranslator : requesting data without being initialized!");
      return false;
    }

    if (name_to_door.find(door_name) == name_to_door.end()) {
      ROS_ERROR_STREAM("Attempted to get through point of non-existent door: " << door_name);
      assert (false);
      return false;
    }
    const auto& door = name_to_door[door_name];

    bwi_mapper::Point2f approach_point;


    float unused_approach_yaw;
    bool door_approachable = getApproachPoint(door_name, current_location, approach_point, unused_approach_yaw);
    if (door_approachable) {
      if (approach_point.x == door.approach_points[0].x &&
          approach_point.y == door.approach_points[0].y) {
        point = door.approach_points[1];
        yaw = door.approach_yaw[1] + M_PI;
      } else {
        point = door.approach_points[0];
        yaw = door.approach_yaw[0] + M_PI;
      }
      return true;
    }

    return false;
  }

  bool BwiLogicalTranslator::isRobotFacingDoor(
          const bwi_mapper::Point2f &current_location,
          float yaw, float threshold, const std::string &door_name) {

    if (!initialized_) {
      ROS_ERROR_STREAM("BwiLogicalTranslator : requesting data without being initialized!");
      return false;
    }

    if (name_to_door.find(door_name) == name_to_door.end()) {
      ROS_ERROR_STREAM("Attempted to check if facing non-existent door: " << door_name);
      assert (false);
      return false;
    }
    const auto& door = name_to_door[door_name];

    const bwi_mapper::Point2f& center_pt = door.door_center;
    if (bwi_mapper::getMagnitude(center_pt - current_location) >
        threshold) {
      return false;
    }

    bwi_mapper::Point2f diff_pt = center_pt - current_location;
    float orientation_to_door = atan2f(diff_pt.y, diff_pt.x);
    while (orientation_to_door > yaw + M_PI) orientation_to_door -= 2*M_PI;
    while (orientation_to_door <= yaw - M_PI) orientation_to_door += 2*M_PI;
    return std::fabs(orientation_to_door - yaw) <= M_PI / 3;

  }

  bool BwiLogicalTranslator::isRobotBesideDoor(
          const bwi_mapper::Point2f &current_location,
          float yaw, float threshold, const std::string &door_name) {

    if (!initialized_) {
      ROS_ERROR_STREAM("BwiLogicalTranslator : requesting data without being initialized!");
      return false;
    }

    if (name_to_door.find(door_name) == name_to_door.end()) {
      ROS_ERROR_STREAM("Attempted to check if beside non-existent door: " << door_name);
      assert (false);
      return false;
    }
    const auto& door = name_to_door[door_name];

    const bwi_mapper::Point2f& center_pt = door.door_center;
    return bwi_mapper::getMagnitude(center_pt - current_location) <= threshold;

  }

  bool BwiLogicalTranslator::isRobotFacingLocation(
          const bwi_mapper::Point2f &current_location,
          float yaw, float threshold, std::string &location_name) {

    if (!initialized_) {
      ROS_ERROR_STREAM("VillaLogicalTranslator : requesting data without being initialized!");
      return false;
    }

    if (location_approach_map_.find(location_name) ==
      location_approach_map_.end()) {
      return false; 
    }

    bwi_mapper::Point2f object_pt(location_approach_map_[location_name].position.x,
                                                        location_approach_map_[location_name].position.y);
    if (bwi_mapper::getMagnitude(object_pt - current_location) >
        threshold) {
      return false;
    }

    float approach_orientation = tf::getYaw(location_approach_map_[location_name].orientation);
    while (approach_orientation > yaw + M_PI) approach_orientation -= 2*M_PI;
    while (approach_orientation <= yaw - M_PI) approach_orientation += 2*M_PI;
    return std::fabs(approach_orientation - yaw) <= M_PI / 3;

  }

  bool BwiLogicalTranslator::isRobotBesideObject(
      const bwi_mapper::Point2f& current_location,
      float yaw, float threshold, std::string& object_name) {

    if (!initialized_) {
      ROS_ERROR_STREAM("VillaLogicalTranslator : requesting data without being initialized!");
      return false;
    }

    if (location_approach_map_.find(object_name) ==
      location_approach_map_.end()) {
      return false; 
    }

    bwi_mapper::Point2f object_pt(location_approach_map_[object_name].position.x,
                                                        location_approach_map_[object_name].position.y);
    return bwi_mapper::getMagnitude(object_pt - current_location) <= threshold;

  }

  size_t BwiLogicalTranslator::getRegionIdx(
      const bwi_mapper::Point2f& region_pt) {

    if (!initialized_) {
      ROS_ERROR_STREAM("VillaLogicalTranslator : requesting data without being initialized!");
      return false;
    }

    bwi_mapper::Point2f grid = bwi_mapper::toGrid(region_pt, info_);
    size_t map_idx = MAP_IDX(info_.width, (int) grid.x, (int) grid.y);
    if (map_idx > region_map_.size()) {
      return (size_t) -1;
    }
    return (size_t) region_map_[map_idx];

  }

  bool BwiLogicalTranslator::getRobotLocation(
          const bwi::Point2f& current_location, 
          float yaw, float threshold, std::string& location_name) {


    location_name = "";

    vector<int> indices;
    vector<float> sq_distances;
    location_tree.nearestKSearch({current_location.x, current_location.y},1, indices, sq_distances);
    if (indices.empty()) {
      return false;
    }
    // If the point is not within our threshold
    if (sqrtf(sq_distances.at(0)) > threshold) {
      location_name = "";
      return false;
    }
    location_name = index_to_name[indices.at(0)];
    return true;
  }

  void BwiLogicalTranslator::initializeStaticCostmapToggleService() {
    ROS_INFO_STREAM("BwiLogicalTranslator: Waiting for static_costmap dyn reconfigure service..");
    static_costmap_toggle_client_ = 
      nh_->serviceClient<dynamic_reconfigure::Reconfigure>("move_base/global_costmap/static_layer/set_parameters");
    static_costmap_toggle_client_.waitForExistence();
    ROS_INFO_STREAM("BwiLogicalTranslator: static_costmap dyn reconfigure service found!");
    static_costmap_toggle_client_initialized_ = true;
  }

  void BwiLogicalTranslator::enableStaticCostmap(bool value) {
    if (!static_costmap_toggle_client_initialized_) {
      initializeStaticCostmapToggleService();
    }
    dynamic_reconfigure::Reconfigure static_costmap_toggle;
    static_costmap_toggle.request.config.bools.resize(1);
    static_costmap_toggle.request.config.bools[0].name = "enabled";
    static_costmap_toggle.request.config.bools[0].value = value;
    static_costmap_toggle_client_.call(static_costmap_toggle);
  }

    bool
    BwiLogicalTranslator::getFacingDoor(const bwi::Point2f &current_location, float yaw, float threshold,
                                        std::string &door_name) {
        vector<int> indices;
        vector<float> sq_distances;
        location_tree.radiusSearch({current_location.x, current_location.y},threshold, indices, sq_distances);
        if (indices.empty()) {
            return false;
        }

        for (const auto index: indices) {
            const auto& name = index_to_name[index];
            // Is this point a door?
            if (name_to_door.find(name) == name_to_door.end()) {
                continue;
            }
            const auto& door = name_to_door[door_name];
            bool is_facing = isRobotFacingDoor(current_location, yaw, threshold, door_name);
            if (is_facing) {
                return true;
            }
        }
        door_name = "";
      return false;
    }

} /* namespace bwi_logical_translator */
