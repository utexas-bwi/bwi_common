#include <boost/foreach.hpp>
#include <bwi_mapper/path_finder.h>
#include <bwi_mapper/map_inflator.h>
#include <bwi_mapper/map_loader.h>
#include <bwi_mapper/map_utils.h>
#include <bwi_msgs/AvailableRobotWithLocationArray.h>
#include <bwi_tools/common/RNG.h>
#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>
#include <tf/transform_datatypes.h>

ros::ServiceClient spawn_model_client;
int total_random_persons;
int launched_random_persons;
std::string map_file;
float person_diameter = 0.5;
float linear_velocity_multiplier = 1.0;
float angular_velocity_multiplier = 1.0;
std::string person_urdf;

ros::ServiceServer pause_service;
ros::ServiceServer unpause_service;
bool global_pause;

nav_msgs::OccupancyGrid map_;
nav_msgs::OccupancyGrid inflated_map_;

std::vector<std::string> models;
std::vector<bool> location_initialized;
std::vector<geometry_msgs::Pose> locations;
std::vector<bool> goal_initialized;
std::vector<bwi_mapper::Point2d> goals;
std::vector<ros::Subscriber> location_subscribers;
std::vector<ros::Publisher> command_publisher;
std::vector<boost::shared_ptr<bwi_mapper::PathFinder> > path_finders;
std::vector<bool> robot_paused;
std::vector<ros::Time> pause_start_times;

ros::Publisher status_publisher;

RNG rng(time(NULL));

bool globalPauseCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
  global_pause = true;
}

bool globalUnpauseCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
  global_pause = false;
}

bwi_mapper::Point2d getPersonLocation(int person_idx, int try_alternates = 0) {
  bwi_mapper::Point2f person_loc_map(locations[person_idx].position.x, locations[person_idx].position.y);
  bwi_mapper::Point2f current_pt_f = bwi_mapper::toGrid(person_loc_map, map_.info);
  bwi_mapper::Point2d current_pt(current_pt_f.x, current_pt_f.y);
  for (int x = -try_alternates; x <= try_alternates; ++x) {
    for (int y = -try_alternates; y <= try_alternates; ++y) {
      bwi_mapper::Point2d test_diff = bwi_mapper::Point2d(x, y);
      bwi_mapper::Point2d test_pt = current_pt + test_diff;
      int idx = test_pt.y * map_.info.width + test_pt.x;
      if (inflated_map_.data[idx] != 100) {
        return test_pt;
      }
    }
  }
  return current_pt;
}

bool sendVelocityCommand(int person_idx) {
  bwi_mapper::Point2d current_pt;
  int alt = 0;
  while(alt < 4) {
    if (alt == 4) {
      std::cout << "person stuck in obstacle. not sure what to do." << std::endl;
    }
    current_pt = getPersonLocation(person_idx, alt);
    int idx = current_pt.y * map_.info.width + current_pt.x;
    if (inflated_map_.data[idx] != 100) {
      break;
    }
    ++alt;
  }
  for (int i = 0; i < 5; ++i) {
    bwi_mapper::Point2d next_pt;
    if (!path_finders[person_idx]->getNextCloserPointToSearchOrigin(current_pt, next_pt)) {
      // Close enough to the goal.
      /* std::cout << "reached goal" << std::endl; */
      robot_paused[person_idx] = true;
      pause_start_times[person_idx] = ros::Time::now();

      // Publish zero velocity.
      geometry_msgs::Twist twist_msg;
      command_publisher[person_idx].publish(twist_msg);
      return false;
    }
    current_pt = next_pt;
  }

  // Now calculate the difference to current_pt.
  bwi_mapper::Point2f interm_loc = bwi_mapper::toMap(current_pt, map_.info);

  float xdiff = interm_loc.x - locations[person_idx].position.x;
  float ydiff = interm_loc.y - locations[person_idx].position.y;
  float orientation = tf::getYaw(locations[person_idx].orientation);
  /* float adiff = 0; */
  float adiff = atan2f(ydiff, xdiff) - orientation;
  while (adiff <= -M_PI) adiff += 2*M_PI;
  while (adiff > M_PI) adiff -= 2*M_PI;

  /* std::cout << xdiff << " " << ydiff << " " << atan2f(ydiff, xdiff) << " " << tf::getYaw(locations[person_idx].orientation) << " " << adiff << std::endl; */

  // if (fabs(adiff > 0.2)) {
  //   xdiff = 0;
  //   ydiff = 0;
  // } else {
    float xdiffnew = xdiff * cosf(orientation) + ydiff * sinf(orientation);
    float ydiffnew = ydiff * cosf(orientation) - xdiff * sinf(orientation);
    xdiff = xdiffnew;
    ydiff = ydiffnew;
  /* } */

  /* std::cout << xdiff << " " << ydiff << " " << adiff << std::endl; */
  geometry_msgs::Twist twist_msg;

  twist_msg.linear.x = linear_velocity_multiplier * xdiff;
  twist_msg.linear.x = std::max(twist_msg.linear.x, -0.5);
  twist_msg.linear.x = std::min(twist_msg.linear.x, 0.5);

  twist_msg.linear.y = linear_velocity_multiplier * ydiff;
  twist_msg.linear.y = std::max(twist_msg.linear.y, -0.5);
  twist_msg.linear.y = std::min(twist_msg.linear.y, 0.5);

  twist_msg.angular.z = angular_velocity_multiplier * adiff;
  twist_msg.angular.z = std::max(twist_msg.angular.z, -0.5);
  twist_msg.angular.z = std::min(twist_msg.angular.z, 0.5);

  command_publisher[person_idx].publish(twist_msg);

  return true;
}

void generateNewGoal(int person_idx) {
  // Generate a reachable goal from the person's current location.

  bwi_mapper::Point2d person_loc_grid = getPersonLocation(person_idx, 3);

  path_finders[person_idx].reset(new bwi_mapper::PathFinder(inflated_map_, person_loc_grid));
  while (true) {
    bwi_mapper::Point2d goal_candidate;
    goal_candidate.x = rng.randomInt(map_.info.width - 1);
    goal_candidate.y = rng.randomInt(map_.info.height - 1);
    /* std::cout << "trying goal candidate: " << goal_candidate << std::endl; */
    // New point needs to be at least 5 meters away.
    if (path_finders[person_idx]->pathExists(goal_candidate) &&
        path_finders[person_idx]->getManhattanDistance(goal_candidate) > 100) {
      goals[person_idx] = goal_candidate;
      path_finders[person_idx].reset(new bwi_mapper::PathFinder(inflated_map_, goals[person_idx]));
      break;
    }
  }

  // std::cout << "  person " << person_idx << " goal generated at " << goals[person_idx] <<
  //   " given current loc " << person_loc_grid << std::endl;

  goal_initialized[person_idx] = true;
}

void runner() {

  ros::Rate r(100);
  while (ros::ok()) {

    /* std::cout << "main loop" << std::endl; */
    ros::spinOnce();

    bwi_msgs::AvailableRobotWithLocationArray status_msg;

    ros::Time current_time = ros::Time::now();

    // Send updated commands to all the automated persons.
    // Publish the names and locations of all persons in a single message.
    for (int i = 0; i < models.size(); ++i) {
      if (!location_initialized[i]) {
        continue;
      }

      if (!goal_initialized[i]) {
        generateNewGoal(i);
      }

      if (!global_pause) {
        if (robot_paused[i]) {
          ros::Duration d = current_time - pause_start_times[i];
          if (d.toSec() > 10) {
            robot_paused[i] = false;
            generateNewGoal(i);
          }
        }

        if (!robot_paused[i]) {
          sendVelocityCommand(i);
        }
      } else {
        // Publish zero velocity.
        geometry_msgs::Twist twist_msg;
        command_publisher[i].publish(twist_msg);
      }

      // Add this robot to the status array.
      bwi_msgs::AvailableRobotWithLocation robot;
      robot.robot.type = bwi_msgs::AvailableRobot::PERSON;
      robot.robot.name = models[i];
      robot.pose = locations[i];
      status_msg.robots.push_back(robot);
    }

    status_publisher.publish(status_msg);
    r.sleep();
  }

}

void odometryHandler(const nav_msgs::Odometry::ConstPtr& odom, int person_idx) {
  locations[person_idx] = odom->pose.pose;
  location_initialized[person_idx] = true;
}

// TODO Will require re-initialization in case of the multimap setup.
int initializeRosCommunication() {

  ros::NodeHandle nh, private_nh("~");

  spawn_model_client = nh.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_urdf_model");
  spawn_model_client.waitForExistence();

  std::vector<std::string> required_parameters;
  if (!private_nh.getParam("num_persons", total_random_persons)) {
    required_parameters.push_back("~num_persons");
  }
  if (!private_nh.getParam("map_file", map_file)) {
    required_parameters.push_back("~map_file");
  }
  if (!private_nh.getParam("person_urdf", person_urdf)) {
    required_parameters.push_back("~person_urdf");
  }

  if (required_parameters.size() != 0) {
    ROS_FATAL_STREAM("random_persons_controller: The following parameters need to be supplied:");
    BOOST_FOREACH(const std::string& param, required_parameters) {
      ROS_FATAL_STREAM("                          " << param);
    }
    return -1;
  }
  bwi_mapper::MapLoader mapper(map_file);
  mapper.getMap(map_);

  bwi_mapper::inflateMap(person_diameter / 2, map_, inflated_map_);

  status_publisher = private_nh.advertise<bwi_msgs::AvailableRobotWithLocationArray>("status", 1);

  pause_service = private_nh.advertiseService("pause", globalPauseCallback);
  unpause_service = private_nh.advertiseService("unpause", globalUnpauseCallback);

  return 0;
}

void launchRandomPersons() {
  ros::NodeHandle nh;

  for (int i = 0; i < total_random_persons; ++i) {
    std::string prefix = "auto_person_";

    gazebo_msgs::SpawnModel spawn;
    spawn.request.model_name = prefix + boost::lexical_cast<std::string>(i);

    ros::param::set("/" + spawn.request.model_name + "/tf_prefix", spawn.request.model_name);

    spawn.request.model_xml = person_urdf;
    spawn.request.robot_namespace = spawn.request.model_name;

    // TODO handle spawn location.
    spawn.request.initial_pose.position.y = 4 + 0.8*i;
    spawn.request.initial_pose.orientation.w = 1.0;

    spawn.response.success = false;
    if (spawn_model_client.call(spawn)) {
      if (spawn.response.success) {
        // Create a odom subscriber, and publish the list of persons.
        models.push_back(spawn.request.model_name);
        location_initialized.push_back(false);
        locations.push_back(geometry_msgs::Pose());
        goal_initialized.push_back(false);
        goals.push_back(bwi_mapper::Point2d());
        locations.push_back(geometry_msgs::Pose());
        location_subscribers.push_back(nh.subscribe<nav_msgs::Odometry>("/" + spawn.request.model_name + "/odom", 1,
                                                                        boost::bind(&odometryHandler, _1, i)));
        command_publisher.push_back(nh.advertise<geometry_msgs::Twist>("/" + spawn.request.model_name + "/cmd_vel", 1));
        path_finders.push_back(boost::shared_ptr<bwi_mapper::PathFinder>());
        robot_paused.push_back(false);
        pause_start_times.push_back(ros::Time());
      } else {
        ROS_WARN_STREAM("Received error message while spawning object: " << spawn.response.status_message);
      }
    } else {
      ROS_ERROR_STREAM("Unable to spawn due to service request failure: " << spawn.request.model_name);
    }
  }
}

int main(int argc, char *argv[]) {
  ros::init (argc, argv, "random_persons_controller");
  initializeRosCommunication();
  launchRandomPersons();
  runner();
  return 0;
}
