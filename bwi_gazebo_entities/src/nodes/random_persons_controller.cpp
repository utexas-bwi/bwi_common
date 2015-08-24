#include <ros/ros.h>

ros::ServiceClient spawn_model_client;
int total_random_persons;
int launched_random_persons;
std::string map_file;
float person_diameter = 0.6;

nav_msgs::OccupancyGrid map_;
nav_msgs::OccupancyGrid inflated_map_;
boost::shared_ptr<bwi_mapper::PathFinder>;

std::vector<std::string> models;
std::vector<geometry_msgs::Pose> locations;
std::vector<bool> location_initialized;

void runner() {
  while (ros::ok()) {
    ros::spinOnce();

    // Publish the names and locations of all persons in a single message.
  }

}

void odometryHandler(

// TODO Will require re-initialization in case of the multimap setup.
int initializeRosCommunication() {

  ros::NodeHandle nh;

  spawn_model_client = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
  spawn_model_client.waitForExistence();

  std::vector<std::string> required_parameters;
  if (!nh.getParam("~num_persons", total_random_persons)) {
    required_parameters.push_back("~num_persons");
  }
  if (!nh.getParam("~map_file", map_file)) {
    required_parameters.push_back("~map_file");
  }

  if (required_parameters.size() != 0) {
    ROS_FATAL_STREAM("random_person_controller: The following parameters need to be supplied:");
    BOOST_FOREACH(const std::string& param, required_parameters) {
      ROS_FATAL_STREAM("                          " << param);
    }
    return -1;
  }
  bwi_mapper::MapLoader mapper(map_file);
  mapper.getMap(map_);

  bwi_mapper::inflateMap(person_diameter/2, map_, inflated_map_);

  return 0;
}

void launchRandomPersons() {
  for (int i = 0; i < total_random_persons; ++i) {
    gazebo_msgs::SpawnModel spawn;
    std::string prefix = "auto_person_";

    spawn.request.model_name = prefix + boost::lexical_cast<std::string>(i);

    spawn.response.success = false;
    if (spawn_model_client.call(spawn)) {
      if (spawn.response.success) {
        // Create a odom subscriber, and publish the list of persons.
        models.push_back(spawn.request.model_name);
        locations.push_back(geometry_msgs::Pose());
        location_initialized.push_back(false);
      } else {
        ROS_WARN_STREAM("Received error message while spawning object: " << spawn.response.status_message);
      }
    } else {
      ROS_ERROR_STREAM("Unable to spawn due to service request failure: " << spawn.request.model_name);
    }
  }
}

int main(int argc, const char *argv[]) {

  return 0;
}

