#include "ros/ros.h"
#include "std_msgs/String.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include <vector>
#include <tf/transform_listener.h>
#include <stdio.h>

actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);
nav_msgs::OccupancyGrid occ_grid_msg;
int x_index;
int y_index;
int random_selection;
double x_coordinate;
double y_coordinate;
std::vector<int> occ_grid;
move_base_msgs::MoveBaseGoal goal;

void map_update_cb(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  occ_grid_msg = *msg;
}

void generate_location() {
  //Generate random locations and verify
  random_selection = rand() % occ_grid_msg.data.size();  
  while(occ_grid[random_selection] != 0) {
    random_selection = rand() % occ_grid_msg.data.size();
  }
  x_index = random_selection %  occ_grid_msg.info.width;
  y_index = random_selection / occ_grid_msg.info.width;
  x_coordinate = x_index * occ_grid_msg.info.resolution;
  y_coordinate = y_index * occ_grid_msg.info.resolution;
}

void handle_movement(const ros::TimerEvent& event) {
  ac.cancelAllGoals();
  generate_location();
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.header.frame_id = "/base_link";
  goal.target_pose.pose.position.x = x_coordinate;
  goal.target_pose.pose.position.y = y_coordinate;
  goal.target_pose.pose.position.z = 0;
  ROS_INFO("SENDING GOAL");
  printf("%f, %f", x_coordinate, y_coordinate);
  ac.sendGoal(goal);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "segbot_wandering");
	ros::NodeHandle n;
  tf::TransformListener tf_l;

  //subscriber for the /visualization_marker
  ros::Subscriber map_sub = n.subscribe("/level_mux/map", 1, map_update_cb);

  ros::Timer timer1 = n.createTimer(ros::Duration(10.0), handle_movement);

  while(ros::ok()) {
    ros::spinOnce();
	}
}