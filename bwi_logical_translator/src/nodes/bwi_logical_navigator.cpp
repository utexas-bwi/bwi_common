/**
 * \file  test_door_detector.cpp
 * \brief  A small tester for the door state detector
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
 * $ Id: 05/06/2013 02:05:01 PM piyushk $
 *
 **/
#include "bwi_logical_navigator.h"


#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib/server/simple_action_server.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/thread/thread.hpp>
#include <bwi_tools/resource_resolver.h>
#include <message_filters/subscriber.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <multi_level_map_msgs/ChangeCurrentLevel.h>
#include <multi_level_map_msgs/LevelMetaData.h>
#include <multi_level_map_msgs/MultiLevelMapData.h>
#include <multi_level_map_utils/utils.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/message_filter.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

BwiLogicalNavigator::BwiLogicalNavigator() :
    robot_x_(0), robot_y_(0), robot_yaw_(0), current_level_id_(""), execute_action_server_started_(false),
    change_level_client_available_(false), robot_controller_available_(false) {

  ROS_INFO("BwiLogicalNavigator: Advertising services!");

  ros::param::param("~door_proximity_distance", door_proximity_distance_, 2.0);
  ros::param::param("~location_proximity_distance", location_proximity_distance_, 2.0);

  // Make sure you publish the default map at least once so that navigation can start up! Ensure this pub is latched.
  ros::NodeHandle private_nh("~");
  navigation_map_publisher_ = private_nh.advertise<nav_msgs::OccupancyGrid>("map", 1, true);

  robot_controller_.reset(
      new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>(
          "move_base", true));

  tf_.reset(new tf::TransformListener);
  odom_subscriber_.reset(new message_filters::Subscriber<nav_msgs::Odometry>);
  odom_subscriber_->subscribe(*nh_, "odom", 5);
  tf_filter_.reset(new tf::MessageFilter<nav_msgs::Odometry>(
      *odom_subscriber_, *tf_, global_frame_id_, 5));
  tf_filter_->registerCallback(
      boost::bind(&BwiLogicalNavigator::odometryHandler, this, _1));

  execute_action_server_.reset(new LogicalNavActionServer(*nh_,
                                                          "execute_logical_action",
                                                          boost::bind(&BwiLogicalNavigator::execute, this, _1),
                                                          false));
  add_object_server_ = nh_->advertiseService("update_object",
                                             &BwiLogicalNavigator::updateObject,
                                             this);

  change_floor_resolution_server_ = nh_->advertiseService("resolve_change_floor",
                                                          &BwiLogicalNavigator::changeFloorResolutionHandler,
                                                          this);

  sense_door_server_ = nh_->advertiseService("sense_door_state",
                                             &BwiLogicalNavigator::senseDoor, this);

  current_level_subscriber_ = nh_->subscribe("level_mux/current_level",
                                             1,
                                             &BwiLogicalNavigator::currentLevelHandler,
                                             this);

  multimap_subscriber_ = nh_->subscribe("map_metadata",
                                        1,
                                        &BwiLogicalNavigator::multimapHandler,
                                        this);

  // Subscribe to the global costmap and Use the local costmap width as a substitute for max raytrace range.
  costmap_subscriber_ = nh_->subscribe("move_base/global_costmap/costmap",
                                       1,
                                       &BwiLogicalNavigator::costmapSubscriber,
                                       this);
  costmap_updates_subscriber_ = nh_->subscribe("move_base/global_costmap/costmap_updates",
                                               1,
                                               &BwiLogicalNavigator::costmapUpdatesSubscriber,
                                               this);
  global_costmap_width_ = -1;
  full_global_costmap_update_ = false;
}

void BwiLogicalNavigator::currentLevelHandler(const multi_level_map_msgs::LevelMetaData::ConstPtr &current_level) {
  if (current_level_id_ != current_level->level_id) {
    std::string resolved_map_file = bwi_tools::resolveRosResource(current_level->map_file);
    ros::param::set("~map_file", resolved_map_file);
    std::string resolved_data_directory = bwi_tools::resolveRosResource(current_level->data_directory);
    ros::param::set("~data_directory", resolved_data_directory);
    if (BwiLogicalTranslator::initialize()) {
      publishNavigationMap();
      // Once the translator is initialized, update the current level id.
      current_level_id_ = current_level->level_id;
    }
  }
}

void BwiLogicalNavigator::costmapSubscriber(const nav_msgs::OccupancyGrid::ConstPtr &costmap) {
  global_costmap_width_ = costmap->info.width;
}

void BwiLogicalNavigator::costmapUpdatesSubscriber(const map_msgs::OccupancyGridUpdate::ConstPtr &costmap_updates) {
  if (global_costmap_width_ != -1 && costmap_updates->width == global_costmap_width_) {
    full_global_costmap_update_ = true;
  }
}

void BwiLogicalNavigator::multimapHandler(const multi_level_map_msgs::MultiLevelMapData::ConstPtr &multimap) {

  // Read in the objects for each level.
  BOOST_FOREACH(const multi_level_map_msgs::LevelMetaData &level, multimap->levels) {
          std::string resolved_data_directory = bwi_tools::resolveRosResource(level.data_directory);
          std::string doors_file = resolved_data_directory + "/doors.yaml";
          std::string locations_file = resolved_data_directory + "/locations.yaml";
          bwi_planning_common::readDoorFile(doors_file, level_to_doors_map_[level.level_id]);
          bwi_planning_common::readLocationFile(locations_file,
                                                level_to_loc_names_map_[level.level_id],
                                                level_to_loc_map_[level.level_id]);
        }

  // Start the change level service client.
  if (!change_level_client_available_) {
    ROS_WARN("BwiLogicalTranslator::multimapHandler: level_mux/change_current_level service is not yet available");
    change_level_client_available_ = ros::service::waitForService("level_mux/change_current_level", ros::Duration(5.0));
    if (change_level_client_available_) {
      change_level_client_ = nh_->serviceClient<multi_level_map_msgs::ChangeCurrentLevel>(
          "level_mux/change_current_level");
    }
  }

}

void BwiLogicalNavigator::publishNavigationMap(bool publish_map_with_doors, bool wait_for_costmap_change) {
  if (wait_for_costmap_change) {
    full_global_costmap_update_ = false;
  }
  if (publish_map_with_doors) {
    navigation_map_publisher_.publish(map_with_doors_);
    last_map_published_with_doors_ = true;
  } else {
    navigation_map_publisher_.publish(map_);
    last_map_published_with_doors_ = false;
  }
  if (wait_for_costmap_change) {
    int counter = 0;
    // TODO: Use a different sleep?
    while (!full_global_costmap_update_ && counter < 100) {
      boost::this_thread::sleep(boost::posix_time::milliseconds(20));
      counter++;
    }
    if (counter == 100) {
      ROS_WARN_STREAM(
          "bwi_logical_navigator: Waited 2 seconds for move_base global costmap to change, but it did not. I'll move forward with the assumption that the map changed and I missed the notification.");
    }
  }
}

void BwiLogicalNavigator::senseState(bwi_msgs::LogicalNavigationState &observations) {

  ROS_INFO_STREAM("sensing state");

  size_t region_idx = getRegionIdx({robot_x_, robot_y_});
  observations.room = getRegionString(region_idx);

  ROS_INFO_STREAM(observations.room);
  getNearbyLocations({robot_x_, robot_y_}, location_proximity_distance_, observations.nearby_locations);

  observations.facing.reserve(observations.nearby_locations.size());
  // Check whether we're facing each of the locations we're near
  std::transform(observations.nearby_locations.begin(),
                 observations.nearby_locations.end(),
                 std::back_inserter(observations.facing),
                 boost::bind(&BwiLogicalTranslator::isRobotFacingLocation,
                             this,
                             bwi::Point2f(robot_x_, robot_y_),
                             robot_yaw_,
                             location_proximity_distance_,
                             _1));


}

bool BwiLogicalNavigator::executeNavigationGoal(
    const geometry_msgs::PoseStamped &pose) {

  if (!robot_controller_available_) {
    ROS_INFO("BwiLogicalNavigator: Need to send command to robot. Waiting for move_base server...");
    robot_controller_->waitForServer();
    ROS_INFO("BwiLogicalNavigator:   move_base server found!");
  }

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose = pose;
  robot_controller_->sendGoal(goal);
  bool navigation_request_complete = false;
  while (!navigation_request_complete) {
    if (execute_action_server_->isPreemptRequested() || !ros::ok()) {
      ROS_INFO("BwiLogicalNavigator: Got pre-empted. Cancelling low level navigation task...");
      robot_controller_->cancelGoal();
      break;
    }
    navigation_request_complete = robot_controller_->waitForResult(ros::Duration(0.5));
  }

  if (navigation_request_complete) {
    actionlib::SimpleClientGoalState state = robot_controller_->getState();
    return state == actionlib::SimpleClientGoalState::SUCCEEDED;
  }

  // If we're here, then we preempted the request ourselves. Let's mark our current request as not successful.
  return false;
}

void BwiLogicalNavigator::odometryHandler(
    const nav_msgs::Odometry::ConstPtr &odom) {
  geometry_msgs::PoseStamped pose_in, pose_out;
  pose_in.header = odom->header;
  pose_in.pose = odom->pose.pose;
  tf_->transformPose(global_frame_id_, pose_in, pose_out);
  robot_x_ = pose_out.pose.position.x;
  robot_y_ = pose_out.pose.position.y;
  //ROS_INFO("OdometryHandler X:%f Y:%f" , robot_x_, robot_y_);

  robot_yaw_ = tf::getYaw(pose_out.pose.orientation);

  if (!execute_action_server_started_) {
    execute_action_server_->start();
    execute_action_server_started_ = true;
  }
}

bool BwiLogicalNavigator::approachDoor(const std::string &door_name,
                                       bwi_msgs::LogicalNavigationState &observations,
                                       std::string &error_message) {
  error_message = "";

  bwi::Point2f approach_pt;
  float approach_yaw = 0;
  bool door_approachable = false;


  publishNavigationMap(true, true);
  door_approachable = getApproachPoint(door_name, bwi::Point2f(robot_x_, robot_y_), approach_pt, approach_yaw);


  if (door_approachable) {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = global_frame_id_;
    pose.pose.position.x = approach_pt.x;
    pose.pose.position.y = approach_pt.y;
    // std::cout << "approaching " << approach_pt.x << "," << approach_pt.y << std::endl;
    tf::quaternionTFToMsg(
        tf::createQuaternionFromYaw(approach_yaw), pose.pose.orientation);
    bool success = executeNavigationGoal(pose);

    // Publish the observable fluents. Since we're likely going to sense the door, make sure the no-doors map was
    // published.
    publishNavigationMap(false, true);
    senseState(observations);

    return success;
  } else {
    // Planning failure

    error_message = "Cannot interact with " + door_name + " from here.";
    return false;
  }

}

bool BwiLogicalNavigator::goThroughDoor(const std::string &door_name, bwi_msgs::LogicalNavigationState &observations,
                                        std::string &error_message) {
  error_message = "";

  bwi::Point2f approach_pt;
  float approach_yaw = 0;
  bool door_approachable = false;

  publishNavigationMap(false, true);
  door_approachable = getThroughDoorPoint(door_name, bwi::Point2f(robot_x_, robot_y_), approach_pt, approach_yaw);
  enableStaticCostmap(false);


  if (door_approachable) {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = global_frame_id_;
    pose.pose.position.x = approach_pt.x;
    pose.pose.position.y = approach_pt.y;
    // std::cout << "approaching " << approach_pt.x << "," << approach_pt.y << std::endl;
    tf::quaternionTFToMsg(
        tf::createQuaternionFromYaw(approach_yaw), pose.pose.orientation);
    bool success = executeNavigationGoal(pose);


    enableStaticCostmap(true);

    // Publish the observable fluents. Since we're likely going to sense the door, make sure the no-doors map was
    // published.
    publishNavigationMap(false, true);
    senseState(observations);

    return success;
  } else {

    enableStaticCostmap(true);
    senseState(observations);

    error_message = "Cannot interact with " + door_name + " from here.";
    return false;
  }

}

bool BwiLogicalNavigator::approachObject(const std::string &object_name,
                                         bwi_msgs::LogicalNavigationState &observations,
                                         std::string &error_message) {

  error_message = "";

  if (location_approach_map_.find(object_name) == location_approach_map_.end()) {
    error_message = object_name + " does not exist.";
    return false;
  }

  if (!isObjectApproachable(object_name, {robot_x_, robot_y_})) {
    error_message = "Cannot interact with " + object_name + " from the robot's current location.";
    return false;
  }

  publishNavigationMap(true);

  geometry_msgs::PoseStamped pose;
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = global_frame_id_;
  pose.pose = location_approach_map_[object_name];
  bool success = executeNavigationGoal(pose);

  senseState(observations);

  return success;
}



bool BwiLogicalNavigator::resolveChangeFloorRequest(const std::string &new_room,
                                                    const std::string &facing_door,
                                                    std::string &floor_name,
                                                    geometry_msgs::PoseWithCovarianceStamped &pose,
                                                    std::string &error_message) {

  // Make sure we can change floors and all arguments are correct.
  if (!change_level_client_available_) {
    //Make an attempt to connect to see if the client has been made avaliable since instantiation before giving up
    change_level_client_available_ = ros::service::waitForService("level_mux/change_current_level", ros::Duration(0.5));
    if (change_level_client_available_) {
      change_level_client_ = nh_->serviceClient<multi_level_map_msgs::ChangeCurrentLevel>(
          "level_mux/change_current_level");
    } else {
      error_message = "BwiLogicalNavigator has not received the multimap. Cannot change floors!";
      return false;
    }
  }
  bool new_room_found = false;
  typedef std::pair<std::string, std::vector<std::string> > Level2LocNamesPair;
  BOOST_FOREACH(const Level2LocNamesPair &level_to_loc, level_to_loc_names_map_) {
          if (std::find(level_to_loc.second.begin(), level_to_loc.second.end(), new_room) !=
              level_to_loc.second.end()) {
            new_room_found = true;
            floor_name = level_to_loc.first;
            break;
          }
        }

  if (!new_room_found) {
    error_message = "Location " + new_room + " has not been defined on any floor!";
    return false;
  }

  if (current_level_id_ == floor_name) {
    error_message = "The robot is already on " + floor_name + " (in which " + new_room + " exists)!";
    return false;
  }

  // Now find the door on this floor that corresponds to facing_door
  bool door_found;
  bwi_planning_common::Door door;
  BOOST_FOREACH(const bwi_planning_common::Door &floor_door, level_to_doors_map_[floor_name]) {
          if (floor_door.name == facing_door) {
            door_found = true;
            door = floor_door;
            break;
          }
        }

  if (!door_found) {
    error_message = "Door " + facing_door + " has not been defined on floor " + floor_name + "!";
    return false;
  }

  bwi::Point2f approach_pt;
  float approach_yaw = 0;
  if (door.approach_names[0] == new_room) {
    approach_pt = door.approach_points[0];
    approach_yaw = door.approach_yaw[0];
  } else if (door.approach_names[1] == new_room) {
    approach_pt = door.approach_points[1];
    approach_yaw = door.approach_yaw[1];
  } else {
    error_message = "Door " + facing_door + " does not connect location " + new_room + "!";
    return false;
  }

  /* Setup return values */
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = global_frame_id_;
  pose.pose.pose.position.x = approach_pt.x;
  pose.pose.pose.position.y = approach_pt.y;
  tf::quaternionTFToMsg(tf::createQuaternionFromYaw(approach_yaw), pose.pose.pose.orientation);
  pose.pose.covariance[0] = 0.1;
  pose.pose.covariance[7] = 0.1f;
  pose.pose.covariance[35] = 0.25f;

  return true;
}

bool BwiLogicalNavigator::changeFloor(const std::string &new_room,
                                      const std::string &facing_door,
                                      bwi_msgs::LogicalNavigationState &observations,
                                      std::string &error_message) {
  //Just making sure it gets called correctly
  ROS_INFO_STREAM("BwiLogicalNavigator: changefloor called");

  error_message = "";

  multi_level_map_msgs::ChangeCurrentLevel srv;
  srv.request.publish_initial_pose = true;

  if (!(resolveChangeFloorRequest(new_room,
                                  facing_door,
                                  srv.request.new_level_id,
                                  srv.request.initial_pose,
                                  error_message))) {
    ROS_ERROR_STREAM("BwiLogicalTranslator::resolveChangeFloorRequest failed. Error message: " << error_message);
    return false;
  }

  if (change_level_client_.call(srv)) {
    if (!srv.response.success) {
      error_message = srv.response.error_message;
      ROS_ERROR_STREAM("BwiLogicalTranslator::changeFloor service call failed. Error message: " << error_message);
      return false;
    }

    // Yea! the call succeeded! Wait for current_level_id_ to be changed once everything gets updated.
    ros::Rate r(1);
    while (current_level_id_ != srv.request.new_level_id) {
      ROS_INFO_STREAM("BwiLogicalTranslator::changeFloor: Waiting for changes to be updated...");
      r.sleep();
    }
    // Publish the observable fluents
    senseState(observations);
    return true;
  } else {
    error_message = "ChangeCurrentLevel service call failed for unknown reason.";
    return false;
  }
}

bool BwiLogicalNavigator::senseDoor(bwi_msgs::CheckBool::Request &req,
                                    bwi_msgs::CheckBool::Response &res) {

  std::string door_name;
  bool is_facing_door = getRobotFacingDoor({robot_x_, robot_y_}, robot_yaw_, 2.0, door_name);
  if (!is_facing_door) {
    ROS_WARN("Called sense door while not facing a door");
    res.value = false;
    return true;
  }

  bool door_open = isDoorOpen(door_name);
  res.value = door_open;
  return true;
}

void BwiLogicalNavigator::execute(const bwi_msgs::LogicalNavGoalConstPtr &goal) {

  bwi_msgs::LogicalNavResult res;

  std::string status;

  if (goal->command.name == "navigate_to") {
    res.success = navigateTo(goal->command.value[0], res.observations,
                             status);
  } else if (goal->command.name == "go_through") {
    res.success = goThroughDoor(goal->command.value[0], res.observations,
                                status);
  } else if (goal->command.name == "changefloor") {
    res.success = changeFloor(goal->command.value[0], goal->command.value[1], res.observations, status);
  } else {
    res.success = true;
    senseState(res.observations);
  }
  ROS_INFO_STREAM("Status: " << status);
  ROS_INFO_STREAM("Observations: " << res.observations);
  if (res.success) {
    execute_action_server_->setSucceeded(res);
  } else if (execute_action_server_->isPreemptRequested()) {
    execute_action_server_->setPreempted(res);
  } else {
    execute_action_server_->setAborted(res);
  }

}

bool BwiLogicalNavigator::changeFloorResolutionHandler(bwi_msgs::ResolveChangeFloor::Request &req,
                                                       bwi_msgs::ResolveChangeFloor::Response &res) {
  res.success = resolveChangeFloorRequest(req.new_room, req.facing_door, res.floor_name, res.pose, res.error_message);
  return true;
}

bool BwiLogicalNavigator::updateObject(bwi_msgs::UpdateObject::Request &req,
                                       bwi_msgs::UpdateObject::Response &res) {
  if (req.type == bwi_msgs::UpdateObjectRequest::UPDATE) {
    location_approach_map_[req.object_name] = req.pose;
    res.success = true;
  } else if (req.type == bwi_msgs::UpdateObjectRequest::REMOVE) {
    std::map<std::string, geometry_msgs::Pose>::iterator it = location_approach_map_.find(req.object_name);
    if (it == location_approach_map_.end()) {
      ROS_ERROR_STREAM("BwiLogicalNavigator::updateObject: object to remove does not exist");
      res.success = false;
    } else {
      location_approach_map_.erase(it);
      res.success = true;
    }
  } else {
    ROS_ERROR_STREAM("BwiLogicalNavigator::updateObject: unknown request type");
    res.success = false;
  }
}

bool BwiLogicalNavigator::navigateTo(const std::string &location_name, bwi_msgs::LogicalNavigationState &observations,
                                     std::string &status) {
  if (is_door(location_name)) {
    return approachDoor(location_name, observations, status);
  } else {
    return approachObject(location_name, observations, status);
  }
  return false;
}


int main(int argc, char *argv[]) {

  ros::init(argc, argv, "bwi_logical_translator");
  ros::NodeHandle nh;

  ROS_INFO("BwiLogicalNavigator: Starting up node...");
  BwiLogicalNavigator handler;
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();
  ROS_INFO("BwiLogicalNavigator: Stopping node.");

  return 0;
}
