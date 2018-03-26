#include "ros/ros.h"
#include "std_msgs/String.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include <vector>
#include <tf/transform_listener.h>
#include <stdio.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ros/console.h>

#define NUM_TAGS 10;

nav_msgs::OccupancyGrid occ_grid_msg;
int x_index;
int y_index;
int random_selection;
double x_coordinate;
double y_coordinate;
bool marker_seen = false;
ar_track_alvar_msgs::AlvarMarkers current_vis_msg;
nav_msgs::MapMetaData map_metadata_msg;

void map_update_cb(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  ROS_INFO("received map msg\n");
  occ_grid_msg = *msg;
}

void vis_cb(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg) {
    current_vis_msg = *msg;
    marker_seen = true;
}

void map_metadata_cb(const nav_msgs::MapMetaData::ConstPtr& msg) {
  ROS_INFO("received MapMetaData msg\n");
  map_metadata_msg = *msg;
}

void generate_location() {
  //Generate random locations and verify
  ROS_INFO("%d\n", map_metadata_msg.width);
  ROS_INFO("got here\n");
  ROS_INFO("size is %d\n", occ_grid_msg.data[3]);
  random_selection = rand() % occ_grid_msg.data.size();
  ROS_INFO("random_selectino is %d \n", random_selection);
  ROS_INFO("right before while loop\n");
  while(occ_grid_msg.data[random_selection] != 0) {
    ROS_INFO("in while loop\n");
    random_selection = rand() % occ_grid_msg.data.size();
  }
  ROS_INFO("out of while loop\n");
  x_index = random_selection %  occ_grid_msg.info.width;
  y_index = random_selection / occ_grid_msg.info.width;
  x_coordinate = x_index * occ_grid_msg.info.resolution;
  y_coordinate = y_index * occ_grid_msg.info.resolution;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "segbot_wandering");
  ros::NodeHandle n;
  tf::TransformListener tf_l;

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);
  move_base_msgs::MoveBaseGoal goal;
  //subscriber for the /visualization_marker
  ros::Subscriber mapdata_sub = n.subscribe("/level_mux/map_metadata", 1, map_metadata_cb);
  ros::Subscriber map_sub = n.subscribe("/level_mux/map", 1, map_update_cb);
  ros::Subscriber marker_sub = n.subscribe("/ar_track_alvar/AlvarMarkers", 1, vis_cb);

  ros::Duration(1.0).sleep();

  while(ros::ok()) {
    ros::spinOnce();
    ros::Time start_time = ros::Time::now();
    ros::Duration ten_secs(10.0);
    generate_location();
    ROS_INFO("x_coordinate: %f\n", x_coordinate);
    ROS_INFO("y_coordinate: %f\n", y_coordinate);
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.header.frame_id = "/base_link";
    goal.target_pose.pose.position.x = x_coordinate;
    goal.target_pose.pose.position.y = y_coordinate;
    goal.target_pose.pose.position.z = 0;
    ROS_INFO("SENDING GOAL");
    ac.sendGoal(goal);
    while(ros::Time::now() - start_time < ten_secs &&
          ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
            if (marker_seen) {
              system("rosrun stop_base p");
            }
          }
    ac.cancelAllGoals();
  }
}