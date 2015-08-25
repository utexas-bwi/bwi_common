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
#include <tf/transform_datatypes.h>

ros::ServiceClient spawn_model_client;
int total_random_persons;
int launched_random_persons;
std::string map_file;
float person_diameter = 0.6;
float linear_velocity_multiplier = 2.0;
float angular_velocity_multiplier = 1.0;
std::string person_urdf;

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

ros::Publisher status_publisher;

bool sendVelocityCommand(int person_idx) {
  bwi_mapper::Point2f person_loc_map(locations[person_idx].position.x, locations[person_idx].position.y);
  bwi_mapper::Point2f current_pt_f = bwi_mapper::toGrid(person_loc_map, map_.info);
  bwi_mapper::Point2d current_pt(current_pt_f.x, current_pt_f.y);
  for (int i = 0; i < 5; ++i) {
    bwi_mapper::Point2d next_pt;
    if (!path_finders[person_idx]->getNextCloserPointToSearchOrigin(current_pt, next_pt)) {
      // Close enough to the goal.
      return false;
    }
    current_pt = next_pt;
  }

  // Now calculate the difference to current_pt.
  bwi_mapper::Point2f interm_loc = bwi_mapper::toMap(current_pt, map_.info);

  float xdiff = interm_loc.x - locations[person_idx].position.x;
  float ydiff = interm_loc.y - locations[person_idx].position.y;
  float adiff = 0;
  /* float adiff = atan2f(ydiff, xdiff) - tf::getYaw(locations[person_idx].orientation); */
  while (adiff <= -M_PI) adiff += 2*M_PI;
  while (adiff > M_PI) adiff -= 2*M_PI;

  std::cout << xdiff << " " << ydiff << " " << adiff << std::endl;
  geometry_msgs::Twist twist_msg;

  twist_msg.linear.x = linear_velocity_multiplier * xdiff;
  twist_msg.linear.x = std::max(twist_msg.linear.x, -0.5);
  twist_msg.linear.x = std::min(twist_msg.linear.x, 0.5);

  twist_msg.linear.y = linear_velocity_multiplier * ydiff;
  twist_msg.linear.y = std::max(twist_msg.linear.y, -0.5);
  twist_msg.linear.y = std::min(twist_msg.linear.y, 0.5);

  twist_msg.angular.z = angular_velocity_multiplier * adiff;
  twist_msg.angular.z = std::max(twist_msg.angular.z, -1.0);
  twist_msg.angular.z = std::min(twist_msg.angular.z, 1.0);

  command_publisher[person_idx].publish(twist_msg);

  return true;
}

void generateNewGoal(int person_idx) {
  // Generate a reachable goal from the person's current location.
  bwi_mapper::Point2f person_loc_map(locations[person_idx].position.x, locations[person_idx].position.y);
  bwi_mapper::Point2f person_loc_grid_f = bwi_mapper::toGrid(person_loc_map, map_.info);
  bwi_mapper::Point2d person_loc_grid(person_loc_grid_f.x, person_loc_grid_f.y);

  path_finders[person_idx].reset(new bwi_mapper::PathFinder(inflated_map_, person_loc_grid));
  RNG rng(time(NULL));
  while (true) {
    bwi_mapper::Point2d goal_candidate;
    goal_candidate.x = rng.randomInt(map_.info.width);
    goal_candidate.y = rng.randomInt(map_.info.height);
    // New point needs to be at least 5 meters away.
    if (path_finders[person_idx]->pathExists(goal_candidate) &&
        path_finders[person_idx]->getManhattanDistance(goal_candidate) > 100) {
      goals[person_idx] = goal_candidate;
      path_finders[person_idx].reset(new bwi_mapper::PathFinder(inflated_map_, goals[person_idx]));
      break;
    }
  }

  std::cout << "  person " << person_idx << " goal generated at " << goals[person_idx] <<
    " given current loc " << person_loc_grid << std::endl;

  goal_initialized[person_idx] = true;
}

void runner() {

  while (ros::ok()) {

    /* std::cout << "main loop" << std::endl; */
    ros::spinOnce();

    bwi_msgs::AvailableRobotWithLocationArray status_msg;

    // Send updated commands to all the automated persons.
    // Publish the names and locations of all persons in a single message.
    for (int i = 0; i < models.size(); ++i) {
      if (!location_initialized[i]) {
        continue;
      }

      if (!goal_initialized[i] || !sendVelocityCommand(i)) {
        generateNewGoal(i);
        sendVelocityCommand(i);
      }

      // Add this robot to the status array.
      bwi_msgs::AvailableRobotWithLocation robot;
      robot.robot.type = bwi_msgs::AvailableRobot::PERSON;
      robot.robot.name = models[i];
      robot.pose = locations[i];
      status_msg.robots.push_back(robot);
    }

    status_publisher.publish(status_msg);
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
  return 0;
}

void launchRandomPersons() {
  ros::NodeHandle nh;

  for (int i = 0; i < total_random_persons; ++i) {
    gazebo_msgs::SpawnModel spawn;
    std::string prefix = "auto_person_";

    spawn.request.model_name = prefix + boost::lexical_cast<std::string>(i);
    spawn.request.model_xml = person_urdf;
    spawn.request.robot_namespace = spawn.request.model_name;

    // TODO handle spawn location.
    spawn.request.initial_pose.position.y = 8 + 2*i;
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

