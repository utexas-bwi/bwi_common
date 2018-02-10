#include "ros/ros.h"
#include "std_msgs/String.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include <vector>
#include <tf/transform_listener.h>

actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);
nav_msgs::OccupancyGrid occ_grid_msg;
int x_index;
int y_index;
int random_selection;
double x_coordinate;
double y_coordinate;
std::vector<int> occ_grid;

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

move_base_msgs::MoveBaseGoal create_goal() {
  //Intiialize values of the Goal message
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.header.frame_id = "/base_link";
  goal.target_pose.pose.position.x = x_coordinate;
  goal.target_pose.pose.position.y = y_coordinate;
  goal.target_pose.pose.position.z = 0;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "segbot_wandering");
	ros::NodeHandle n;
  tf::TransformListener tf_l;

  //subscriber for the /visualization_marker
  ros::Subscriber map_sub = n.subscribe("/level_mux/map", 1, map_update_cb);

  while(ros::ok()) {
    ros::spinOnce();
    generate_location();
    move_base_msgs::MoveBaseGoal cur_goal = create_goal();
    ROS_INFO("SENDING GOAL");
    ac.sendGoal(cur_goal);
    //Loop to find Tag
      //Approach Tag
    //Resume and Loop
	}
}