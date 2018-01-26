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

#define SCALAR 1.1

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
      required_parameters.push_back("~map_file");
    }
    if (!ros::param::get("~data_directory", data_directory)) {
      required_parameters.push_back("~data_directory");
    }
    if (required_parameters.size() != 0) {
      std::string message = "BwiLogicalTranslator: Required parameters [" +
        boost::algorithm::join(required_parameters, ", ") + "] missing!";
      ROS_FATAL_STREAM(message);
      throw std::runtime_error(message);
    }

    std::string door_file = bwi_planning_common::getDoorsFileLocationFromDataDirectory(data_directory);
    ROS_INFO_STREAM("BwiLogicalTranslator: Reading door file: " + door_file);
    bwi_planning_common::readDoorFile(door_file, doors_);

    std::string location_file = bwi_planning_common::getLocationsFileLocationFromDataDirectory(data_directory);
    ROS_INFO_STREAM("BwiLogicalTranslator: Reading locations file: " + location_file);
    bwi_planning_common::readLocationFile(location_file, locations_, location_map_);

    std::string object_file = bwi_planning_common::getObjectsFileLocationFromDataDirectory(data_directory);
    ROS_INFO_STREAM("BwiLogicalTranslator: Checking if objects file exists: " + object_file);
    if (boost::filesystem::exists(object_file)) {
      ROS_INFO_STREAM("BwiLogicalTranslator:   Objects file exists. Reading now!");
      bwi_planning_common::readObjectApproachFile(object_file, object_approach_map_);
    }

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
    object_approachable_space_.clear();

    initialized_ = true;
    return true;
  }

  bool BwiLogicalTranslator::isDoorOpen(size_t idx) {

    if (!initialized_) {
      ROS_ERROR_STREAM("BwiLogicalTranslator : requesting data without being initialized!");
      return false;
    }

    if (idx > doors_.size()) {
      return false;
    }

    enableStaticCostmap(false);
    // TODO: this should not be necesary, since we make service calls 
    boost::this_thread::sleep(boost::posix_time::milliseconds(250));

    bwi_mapper::Point2f start_pt, goal_pt;
    float start_yaw, goal_yaw;

    start_pt = doors_[idx].approach_points[0];
    goal_pt = doors_[idx].approach_points[1];
    start_yaw = doors_[idx].approach_yaw[0];
    goal_yaw = doors_[idx].approach_yaw[1];

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
        if (srv.response.plan.poses.size() != 0) {
          // Valid plan received. Check if plan distance seems reasonable
          float distance = 0;
          geometry_msgs::Point old_pt =
            srv.response.plan.poses[0].pose.position;
          for (size_t i = 1; i < srv.response.plan.poses.size(); ++i) {
            geometry_msgs::Point current_pt =
              srv.response.plan.poses[i].pose.position;
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

    if (counter == 3) {
      // we have see the door open 3 consequitive times
      return true;
    } else {
      return false;
    }
  }

  bool BwiLogicalTranslator::getApproachPoint(size_t idx,
      const bwi_mapper::Point2f& current_location,
      bwi_mapper::Point2f& point, float &yaw) {

    if (!initialized_) {
      ROS_ERROR_STREAM("BwiLogicalTranslator : requesting data without being initialized!");
      return false;
    }

    if (idx > doors_.size()) {
      return false;
    }

    // See if we've calculated the approachable space for this door.
    if (door_approachable_space_1_.find(idx) == door_approachable_space_1_.end()) {
      const bwi_planning_common::Door& door = doors_[idx];
      const bwi_mapper::Point2d approach_pt_1(bwi_mapper::toGrid(door.approach_points[0], info_));
      door_approachable_space_1_[idx] = boost::shared_ptr<bwi_mapper::PathFinder>(new bwi_mapper::PathFinder(inflated_map_with_doors_, approach_pt_1));
    }
    if (door_approachable_space_2_.find(idx) == door_approachable_space_2_.end()) {
      const bwi_planning_common::Door& door = doors_[idx];
      const bwi_mapper::Point2d approach_pt_2(bwi_mapper::toGrid(door.approach_points[1], info_));
      door_approachable_space_2_[idx] = boost::shared_ptr<bwi_mapper::PathFinder>(new bwi_mapper::PathFinder(inflated_map_with_doors_, approach_pt_2));
    }

    // Find the approach point to which we can find a path. If both approach points can be reached, fine the approach
    // point which is closer.
    bwi_mapper::Point2d grid(bwi_mapper::toGrid(current_location, info_));
    int distance_1 = door_approachable_space_1_[idx]->getManhattanDistance(grid);
    int distance_2 = door_approachable_space_2_[idx]->getManhattanDistance(grid);
    if (distance_1 >= 0 || distance_2 >= 0) {
      if (distance_1 >= 0 && (distance_1 < distance_2 || distance_2 < 0)) {
        point = doors_[idx].approach_points[0];
        yaw = doors_[idx].approach_yaw[0];
      } else {
        point = doors_[idx].approach_points[1];
        yaw = doors_[idx].approach_yaw[1];
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

    if (object_approachable_space_.find(object_name) == object_approachable_space_.end()) {
      const geometry_msgs::Pose& object_pose = object_approach_map_[object_name];
      const bwi_mapper::Point2d approach_pt(bwi_mapper::toGrid(bwi_mapper::Point2f(object_pose.position.x,
                                                                                   object_pose.position.y),
                                                               info_));
      object_approachable_space_[object_name] = boost::shared_ptr<bwi_mapper::PathFinder>(new bwi_mapper::PathFinder(inflated_map_with_doors_, approach_pt));
    }

    bwi_mapper::Point2d grid_pt(bwi_mapper::toGrid(current_location, info_));
    return object_approachable_space_[object_name]->pathExists(grid_pt);
  }

  bool BwiLogicalTranslator::getThroughDoorPoint(size_t idx,
      const bwi_mapper::Point2f& current_location,
      bwi_mapper::Point2f& point, float& yaw) {

    if (!initialized_) {
      ROS_ERROR_STREAM("BwiLogicalTranslator : requesting data without being initialized!");
      return false;
    }

    if (idx > doors_.size()) {
      return false;
    }

    bwi_mapper::Point2f approach_point;

    float unused_approach_yaw;
    bool door_approachable = getApproachPoint(idx, current_location, approach_point, unused_approach_yaw);
    if (door_approachable) {
      if (approach_point.x == doors_[idx].approach_points[0].x &&
          approach_point.y == doors_[idx].approach_points[0].y) {
        point = doors_[idx].approach_points[1];
        yaw = doors_[idx].approach_yaw[1] + M_PI;
      } else {
        point = doors_[idx].approach_points[0];
        yaw = doors_[idx].approach_yaw[0] + M_PI;
      }
      return true;
    }

    return false;
  }

  bool BwiLogicalTranslator::isRobotFacingDoor(
      const bwi_mapper::Point2f& current_location,
      float yaw, float threshold, size_t idx) {

    if (!initialized_) {
      ROS_ERROR_STREAM("BwiLogicalTranslator : requesting data without being initialized!");
      return false;
    }

    bwi_mapper::Point2f& center_pt = doors_[idx].door_center;
    if (bwi_mapper::getMagnitude(center_pt - current_location) >
        threshold) {
      return false;
    }

    bwi_mapper::Point2f diff_pt = center_pt - current_location;
    float orientation_to_door = atan2f(diff_pt.y, diff_pt.x);
    while (orientation_to_door > yaw + M_PI) orientation_to_door -= 2*M_PI;
    while (orientation_to_door <= yaw - M_PI) orientation_to_door += 2*M_PI;
    if (fabs(orientation_to_door - yaw) > M_PI / 3) {
      return false;
    }

    return true;
  }

  bool BwiLogicalTranslator::isRobotBesideDoor(
      const bwi_mapper::Point2f& current_location,
      float yaw, float threshold, size_t idx) {

    if (!initialized_) {
      ROS_ERROR_STREAM("BwiLogicalTranslator : requesting data without being initialized!");
      return false;
    }

    bwi_mapper::Point2f& center_pt = doors_[idx].door_center;
    if (bwi_mapper::getMagnitude(center_pt - current_location) >
        threshold) {
      return false;
    }

    return true;
  }

  size_t BwiLogicalTranslator::getLocationIdx(
      const bwi_mapper::Point2f& current_location) {

    if (!initialized_) {
      ROS_ERROR_STREAM("BwiLogicalTranslator : requesting data without being initialized!");
      return false;
    }

    bwi_mapper::Point2f grid = bwi_mapper::toGrid(current_location, info_);
    size_t map_idx = MAP_IDX(info_.width, (int) grid.x, (int) grid.y);
    if (map_idx > location_map_.size()) {
      return (size_t) -1;
    }
    return (size_t) location_map_[map_idx];

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

} /* namespace bwi_logical_translator */
