#include "ChickenSafety.h"
#include <math.h>
#include <stdlib.h>
#include <time.h>

ChickenSafetyProtocol::ChickenSafetyProtocol(string ROBOT_NAME, string HOST_NAME){
  this->client_ = new ASPClient(ROBOT_NAME + "/action_executor/execute_plan", true);
  this->lac_ = new LAClient(ROBOT_NAME + "/execute_logical_goal", true);
  stop = nh_.advertise<actionlib_msgs::GoalID>(ROBOT_NAME + "/move_base/cancel", 1, true);
  this->makePlanService = nh_.serviceClient<nav_msgs::GetPlan>(ROBOT_NAME + "/move_base/NavfnROS/make_plan");
  this->stop_rate = new ros::Rate(100);

  this->client_->waitForServer();
  this->lac_->waitForServer();
  this->makePlanService.waitForExistence();

  string commonEBand = "/move_base/EBandPlannerROS/eband_visualization_array";
  string myEBandTopic = ROBOT_NAME + commonEBand;

  string othersEBandTopic = (ROBOT_NAME==""?"/others":"/roberto") + commonEBand;
  this->subscribeMyPlan_ = nh_.subscribe(myEBandTopic, 1, &ChickenSafetyProtocol::subscribeMyPlan, this);
  this->subscribeOthersPlan_ = nh_.subscribe(othersEBandTopic, 1, &ChickenSafetyProtocol::subscribeOthersPlan, this);

  this->spinner = new ros::AsyncSpinner(2);
  this->spinner->start();

  // Read predefined parking zones
  string loc_file = "/home" + HOST_NAME + "/catkin_ws/src/bwi_common/utexas_gdc/maps/simulation/3ne/objects.yaml";
  bwi_planning_common::readObjectApproachFile(loc_file, parkingZones_);

  //ROS_INFO_STREAM("Number of predefined parking zones: " << parkingZones_.size());
  //for(map<string, geometry_msgs::Pose>::iterator iter = parkingZones_.begin(); iter != parkingZones_.end(); iter++) ROS_INFO_STREAM(iter->first);

  if(ROBOT_NAME == "") global_frame_id_ = "level_mux_map";
  else global_frame_id_ = "marvin/level_mux_map";
};
void ChickenSafetyProtocol::subscribeMyPlan(const visualization_msgs::MarkerArray plan){
  this->myPlan_.clear();
  for(int i=0; i<plan.markers.size(); i++){
    this->myPlan_.push_back(plan.markers[i].pose);
  }
};
void ChickenSafetyProtocol::subscribeOthersPlan(const visualization_msgs::MarkerArray plan){
  this->othersPlan_.clear();
  for(int i=0; i<plan.markers.size(); i++) othersPlan_.push_back(plan.markers[i].pose);
};
float ChickenSafetyProtocol::dist(geometry_msgs::Point my_plan_pt, geometry_msgs::Point others_plan_pt){
  return hypot(my_plan_pt.x - others_plan_pt.x, my_plan_pt.y - others_plan_pt.y);
};
bool ChickenSafetyProtocol::distanceBaseAlarm(float warningDistance = 0.6){
  bool danger = false;
  if(myPlan_.size() == 0 || othersPlan_.size() == 0) return danger;

  for(int i=0; i<myPlan_.size()||i<othersPlan_.size(); i++){
    if(i<myPlan_.size()){
      if(dist(myPlan_[i].position, othersPlan_[0].position) < warningDistance){
        //ROS_INFO_STREAM("Roberto--->Marvin");
        //ROS_INFO_STREAM("my pose : ("<<myPlan_[i].position.x<<", "<<myPlan_[i].position.y<<")");
        //ROS_INFO_STREAM("others pose : ("<<othersPlan_[0].position.x<<", "<<othersPlan_[0].position.y<<")");
        danger = true;
        return danger;
      }
    }
    if(i<othersPlan_.size()){
      if(dist(myPlan_[0].position, othersPlan_[i].position) < warningDistance){
        //ROS_INFO_STREAM("Roberto<---Marvin");
        //ROS_INFO_STREAM("my pose : ("<<myPlan_[0].position.x<<", "<<myPlan_[0].position.y<<")");
        //ROS_INFO_STREAM("others pose : ("<<othersPlan_[i].position.x<<", "<<othersPlan_[i].position.y<<")");
        danger = true;
        return danger;
      }
    }
    if(i<myPlan_.size() && i<othersPlan_.size()){
      if(dist(myPlan_[i].position, othersPlan_[i].position) < warningDistance){
        //ROS_INFO_STREAM("Roberto-><-Marvin");
        //ROS_INFO_STREAM("my pose : ("<<myPlan_[i].position.x<<", "<<myPlan_[i].position.y<<")");
        //ROS_INFO_STREAM("others pose : ("<<othersPlan_[i].position.x<<", "<<othersPlan_[i].position.y<<")");
        danger = true;
        return danger;
      }
    }
  }

  return danger;
};
void ChickenSafetyProtocol::stopASPPlan(){
  // cancel current ASP planner
  this->client_->cancelGoal();
  // Manually stop the move base.
  actionlib_msgs::GoalID stop_msg;
  stop_msg.id = "";
  stop_rate->sleep();
  stop.publish(stop_msg);
};
float ChickenSafetyProtocol::distance(geometry_msgs::Point start_pt, geometry_msgs::Point goal_pt){
  float distance = 0;
  nav_msgs::GetPlan srv;
  geometry_msgs::PoseStamped &start = srv.request.start;
  geometry_msgs::PoseStamped &goal  = srv.request.goal;
  // set GetPlan service parameters
  start.header.frame_id = goal.header.frame_id = global_frame_id_;
  start.header.stamp    = goal.header.stamp    = ros::Time::now();

  start.pose.position.x = start_pt.x;
  start.pose.position.y = start_pt.y;
  start.pose.position.z = 0;
  start.pose.orientation.w = 1.0;

  goal.pose.position.x = goal_pt.x;
  goal.pose.position.y = goal_pt.y;
  goal.pose.position.z = 0;
  goal.pose.orientation.w = 1.0;

  srv.request.tolerance = 0.5 + 1e-6;

  geometry_msgs::Pose prev_pose = start.pose;
  geometry_msgs::Pose curr_pose = start.pose;

  if(makePlanService.call(srv)) {
    if(srv.response.plan.poses.size() != 0){
      for(int i = 0 ; i<srv.response.plan.poses.size() ; i++){
        curr_pose = srv.response.plan.poses[i].pose;
        if(abs(prev_pose.position.x - curr_pose.position.x) > 1 || abs(prev_pose.position.y - curr_pose.position.y)>1){
          distance = distance + dist(prev_pose.position, curr_pose.position);
          prev_pose = curr_pose;
        }
      }
      distance = distance + dist(prev_pose.position, curr_pose.position);
    }else{
      distance = dist(start_pt, goal_pt);
    }
  }
  else{
    ROS_INFO_STREAM("Planning failed!");
  }
  return distance;
}
string ChickenSafetyProtocol::findSafeZone(){
  string safeZone = "empty";
  string nearZone = "empty";
  float safeDist = 999.f;
  float nearDist = 999.f;

  map<string, geometry_msgs::Pose>::iterator iter;
  for(iter = parkingZones_.begin(); iter != parkingZones_.end(); iter++){
    float my_distance_to_safe_zone = distance(myPlan_.begin()->position, iter->second.position);
    float others_distance_to_safe_zone = distance(othersPlan_.begin()->position, iter->second.position);
    if(my_distance_to_safe_zone < others_distance_to_safe_zone && my_distance_to_safe_zone < safeDist){
      safeDist = my_distance_to_safe_zone;
      safeZone = iter->first;
    }
    if(my_distance_to_safe_zone < nearDist){
      nearDist = my_distance_to_safe_zone;
      nearZone = iter->first;
    }
  }
  if(safeZone == "empty"){
    safeZone = nearZone;
  }
  //ROS_INFO_STREAM("nearest Safe Zone : "<<safeZone);
  return safeZone;
};
void ChickenSafetyProtocol::resume(string location){
  ROS_INFO_STREAM("resume to "<<location);
  plan_execution::ExecutePlanGoal goal;
  plan_execution::AspRule rule;
  plan_execution::AspFluent fluent;

  fluent.name = "not facing";
  fluent.variables.push_back(location);
  rule.body.push_back(fluent);
  goal.aspGoal.push_back(rule);

  this->client_->sendGoal(goal);
};
void ChickenSafetyProtocol::goToSafeZone(string safeZone){
  // safeZone must be defined as object.
  vector<string> parameters;
  bwi_msgs::LogicalActionGoal goal;

  goal.command.name = "goto";
  parameters.push_back(safeZone);
  parameters.push_back("1");
  goal.command.value = parameters;

  this->lac_->sendGoal(goal);
  parameters.clear();
};
// Original waitUntilSafe module

bool ChickenSafetyProtocol::waitUntilSafe(){
  // return true if it is safe, otherwise return false;
  bool safe = false;
  // if robots are 5 meters away
  vector<geometry_msgs::Pose>::iterator my_iter = myPlan_.begin();
  vector<geometry_msgs::Pose>::iterator others_iter = othersPlan_.end();

  float distance_between_robot = dist(my_iter->position, (--others_iter)->position);
  //ROS_INFO_STREAM("waitUntilSafe init");
  if(distance_between_robot > 8.0f){
    //ROS_INFO_STREAM("Robots are 8.0 M away!");
    safe = true;
    return safe;
  }
  //ROS_INFO_STREAM("waitUntilSafe distance between robots");
  if(othersPlan_.size() == 2){
    // Other robot is stopped and planning for next location.
    return safe;
  }
  // if other robot is moving away.
  bool isMovingAway = true;

  others_iter = othersPlan_.begin();
  float prev_dist = dist(my_iter->position, others_iter->position);
  float curr_dist;
  others_iter = ++others_iter;
  //ROS_INFO_STREAM("waitUntilSafe far away init");
  //ROS_INFO_STREAM("Other plan size: "<<othersPlan_.size());
  int index = 1;
  for(; others_iter < othersPlan_.end(); others_iter++){
    // write distance
    //ROS_INFO_STREAM("index : "<<index);
    index = index+1;
    curr_dist = dist(my_iter->position, others_iter->position);
    if(curr_dist<prev_dist){
      isMovingAway = false;
      break;
    }
  }
  //ROS_INFO_STREAM("waitUntilSafe far away finished");
  if(isMovingAway == true){
    //ROS_INFO_STREAM("Other robot is definately planning to move away!");
    safe = true;
  }

  return safe;
};

void ChickenSafetyProtocol::run(string location){
  //ROS_INFO_STREAM("Initiate Safety Protocol");
  bool danger = distanceBaseAlarm();
  if(danger == true){
    //stop plan
    //ROS_INFO_STREAM("Stop plan");
    stopASPPlan();
    // find near safezone
    string safeZone;
    safeZone = findSafeZone();
    // move to safe zone
    ROS_INFO_STREAM("Going to safe zone");
    goToSafeZone(safeZone);
    // Decide when to resume robot.
    //double stime = clock();
    while(ros::ok()){
      //if((clock()-stime)/CLOCKS_PER_SEC > 600) break;
      // Handel ABORTED from bwi logical action
      if(this->lac_->getState() == actionlib::SimpleClientGoalState::ABORTED){
        ROS_INFO_STREAM("Execute Logical Goal failed.");
        ROS_INFO_STREAM("Re-executing.");
        goToSafeZone(safeZone);
      }
      //wait until it is safe.
      if(waitUntilSafe()) break;
      stop_rate->sleep();
    }
    // resume to original goal
    if(this->lac_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
      this->lac_->cancelGoal();
    }
    resume(location);
    while(ros::ok()){
      if(this->client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED) break;
      ROS_INFO_STREAM("waiting for robot to resume");
      this->stop_rate->sleep();
    }
  }
  for(int i=0; i<10; i++) this->stop_rate->sleep();
  return;
};

void ChickenSafetyProtocol::goTo(string location){
  ROS_INFO_STREAM("Going to "<<location);
  plan_execution::ExecutePlanGoal goal;
  plan_execution::AspRule rule;
  plan_execution::AspFluent fluent;

  fluent.name = "not facing";
  fluent.variables.push_back(location);
  rule.body.push_back(fluent);
  goal.aspGoal.push_back(rule);

  this->client_->sendGoal(goal);
  while(ros::ok()){
    if(this->client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED) break;
  }
};
