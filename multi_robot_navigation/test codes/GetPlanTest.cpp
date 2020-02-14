// GetPlan in move_base/NavfnROS/make_plan ONLY works when MOVE_BASE is NOT actually MOVING!
// 1. Make virtual move_base for planning only.
// 2. Use different Method.
// -> Grab plan information from rviz. Subscribe to "move_base/EBandPlannerROS/eband_visualization_array" to grab info.
// See DistModuleTest.cpp for more information.

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <bwi_msgs/LogicalActionAction.h>
#include <plan_execution/ExecutePlanAction.h>
#include <stdlib.h>
#include <time.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/GetPlan.h>
typedef actionlib::SimpleActionClient<plan_execution::ExecutePlanAction> Client;

using namespace std;
bool RANDOM_MOVE = false;
bool roberto_success = true;
bool marvin_success = true;

int marvin_goal = 0;
int roberto_goal = 0;

Client* client;
Client* client2;
actionlib::SimpleActionClient<bwi_msgs::LogicalActionAction>* lnac;

ros::ServiceClient GetPlanClient;
geometry_msgs::PoseWithCovariance my_pose;
geometry_msgs::PoseWithCovariance others_pose;
void callback1(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr pose){
  my_pose = pose->pose;
}
void callback2(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr pose){
  others_pose = pose->pose;
}
void executePlan(string location){
  if(roberto_success == true){
    ROS_INFO_STREAM("Roberto going to " << location);

    plan_execution::ExecutePlanGoal goal;
    plan_execution::AspRule rule;
    plan_execution::AspFluent fluent;

    fluent.name = "not facing";
    fluent.variables.push_back(location);
    rule.body.push_back(fluent);
    goal.aspGoal.push_back(rule);

    ROS_INFO("sending goal");
    client->sendGoal(goal);
    roberto_success = false;
  }
  if(client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    roberto_success = true;
    roberto_goal = (roberto_goal + 1)%10;
  }
}
void GetPlan(ros::NodeHandle *nh){
  nav_msgs::GetPlan srv;
  GetPlanClient = nh->serviceClient<nav_msgs::GetPlan>("/roberto/move_base/NavfnROS/make_plan");
// Size of plan pose = 2
//  GetPlanClient = nh->serviceClient<nav_msgs::GetPlan>("roberto/move_base/NavfnROS/make_plan");
// service [/test/roberto/move_base/NavfnROS/make_plan] has not been advertised  ERROR.
// BUT, this does works with multi_robot_navigation_clever_roberto_v3.
// Maybe nh("~") makes problem?
  GetPlanClient.waitForExistence();
  geometry_msgs::PoseStamped &start = srv.request.start;
  geometry_msgs::PoseStamped &goal  = srv.request.goal;

  start.header.frame_id = goal.header.frame_id = "roberto/level_mux_map";
  start.header.stamp    = goal.header.stamp    = ros::Time::now();

  start.pose.position.x = my_pose.pose.position.x;
  start.pose.position.y = my_pose.pose.position.y;
  start.pose.position.z = 0;
  start.pose.orientation.w = my_pose.pose.orientation.w;

  goal.pose.position.x = others_pose.pose.position.x;
  goal.pose.position.y = others_pose.pose.position.y;
  goal.pose.position.z = 0;
  goal.pose.orientation.w = others_pose.pose.orientation.w;

  srv.request.tolerance = 0.5+1e-6;
  ROS_INFO_STREAM("Plan from MY POSE:"<<my_pose.pose.position.x<<", "<<my_pose.pose.position.y);
  ROS_INFO_STREAM("   To OTHERS POSE:"<<others_pose.pose.position.x<<", "<<others_pose.pose.position.y);
  if(GetPlanClient.call(srv)){
    ROS_INFO_STREAM("Plan size : " << srv.response.plan.poses.size());
  }
}
int main(int argc, char**argv){

  ros::init(argc, argv, "test");
  ros::NodeHandle nh;
  ros::Rate r(1);
  client = new Client("roberto/action_executor/execute_plan", true);
  client2 = new Client("marvin/action_executor/execute_plan", true);
  client->waitForServer();

  vector<string> locations;
  locations.push_back("d3_414a2");
  locations.push_back("d3_414b1");
  locations.push_back("d3_414b2");
  locations.push_back("d3_414a1");

  ros::Subscriber sub1 = nh.subscribe("/roberto/amcl_pose",1,&callback1);
  ros::Subscriber sub2 = nh.subscribe("/marvin/amcl_pose",1,&callback2);
  ros::AsyncSpinner spinner(0);
  spinner.start();

  //
  string location = locations.at(0);
  ROS_INFO_STREAM("Marvin going to " << location);

  plan_execution::ExecutePlanGoal goal;
  plan_execution::AspRule rule;
  plan_execution::AspFluent fluent;

  fluent.name = "not facing";
  fluent.variables.push_back(location);
  rule.body.push_back(fluent);
  goal.aspGoal.push_back(rule);

  ROS_INFO("sending goal");
  client2->sendGoal(goal);
  //
  while(ros::ok){
    GetPlan(&nh);
    executePlan(locations.at(roberto_goal));
    if(roberto_success) break;
  }
  executePlan(locations.at(roberto_goal+2));
  for(int i=0; i<100; i++){
    GetPlan(&nh);
    r.sleep();
  }
  return 0;
}
