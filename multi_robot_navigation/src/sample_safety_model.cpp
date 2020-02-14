#include <actionlib/client/simple_action_client.h>
#include <plan_execution/ExecutePlanAction.h>
#include <bwi_msgs/LogicalActionAction.h>

#include <nav_msgs/GetPlan.h>

#include <visualization_msgs/MarkerArray.h>

#include <ros/ros.h>

#include <bwi_planning_common/structures.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <boost/circular_buffer.hpp>

typedef actionlib::SimpleActionClient<plan_execution::ExecutePlanAction> ASPClient;
typedef actionlib::SimpleActionClient<bwi_msgs::LogicalActionAction> LAClient;

using namespace std;

bool DEBUG = false;

bool REVERSE = true;
string ROBOT_NAME = "/roberto";
string HOST_NAME  = "/jinsoo";
int buffer_size = 10;

class safetyProtocol{
protected:
  ASPClient *client_;
  LAClient *lac_;
  ros::Publisher stop;
  ros::Rate *stop_rate;

  ros::NodeHandle nh_;
  vector<geometry_msgs::Pose> myPlan_;
  vector<geometry_msgs::Pose> othersPlan_;
  boost::circular_buffer<geometry_msgs::Pose> othersHistory_;
  ros::Subscriber subscribeMyPlan_;
  ros::Subscriber subscribeOthersPlan_;
  ros::AsyncSpinner *spinner;

  map<string, geometry_msgs::Pose> parkingZones_;
  ros::ServiceClient makePlanService;
  string global_frame_id_;
  // private functions
  void subscribeMyPlan(const visualization_msgs::MarkerArray plan);
  void subscribeOthersPlan(const visualization_msgs::MarkerArray plan);
  float dist(geometry_msgs::Point my_plan_pt, geometry_msgs::Point others_plan_pt);
  float distance(const geometry_msgs::Point start_pose, const geometry_msgs::Point goal_pose);
  string findSafeZone();
  bool waitUntilSafe();
  bool distanceBaseAlarm(float warningDistance);
public:
  safetyProtocol(string ROBOT_NAME);
  void stopASPPlan();
  void goToSafeZone(string location);
  void resume(string location);
  void run(string location);
  void goTo(string location);
  bool isClientArrive(){
    if(this->client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) return true;
    return false;
  }
};
safetyProtocol::safetyProtocol(string ROBOT_NAME){
  this->client_ = new ASPClient(ROBOT_NAME + "/action_executor/execute_plan", true);
  this->lac_ = new LAClient(ROBOT_NAME + "/execute_logical_goal", true);
  stop = nh_.advertise<actionlib_msgs::GoalID>(ROBOT_NAME + "/move_base/cancel", 1, true);
  this->makePlanService = nh_.serviceClient<nav_msgs::GetPlan>(ROBOT_NAME + "/move_base/NavfnROS/make_plan");
  this->stop_rate = new ros::Rate(100);

  this->othersHistory_ = boost::circular_buffer<geometry_msgs::Pose>(buffer_size);

  this->client_->waitForServer();
  this->lac_->waitForServer();
  this->makePlanService.waitForExistence();

  string commonEBand = "/move_base/EBandPlannerROS/eband_visualization_array";
  string myEBandTopic = ROBOT_NAME + commonEBand;

  string othersEBandTopic = (ROBOT_NAME==""?"/others":"/marvin") + commonEBand;
  this->subscribeMyPlan_ = nh_.subscribe(myEBandTopic, 1, &safetyProtocol::subscribeMyPlan, this);
  this->subscribeOthersPlan_ = nh_.subscribe(othersEBandTopic, 1, &safetyProtocol::subscribeOthersPlan, this);

  this->spinner = new ros::AsyncSpinner(2);
  this->spinner->start();

  // Read predefined parking zones
  string loc_file = "/home" + HOST_NAME + "/catkin_ws/src/bwi_common/utexas_gdc/maps/simulation/3ne/objects.yaml";
  bwi_planning_common::readObjectApproachFile(loc_file, parkingZones_);

  ROS_INFO_STREAM("Number of predefined parking zones: " << parkingZones_.size());
  for(map<string, geometry_msgs::Pose>::iterator iter = parkingZones_.begin(); iter != parkingZones_.end(); iter++) ROS_INFO_STREAM(iter->first);

  if(ROBOT_NAME == "") global_frame_id_ = "level_mux_map";
  else global_frame_id_ = "roberto/level_mux_map";
};
void safetyProtocol::subscribeMyPlan(const visualization_msgs::MarkerArray plan){
  this->myPlan_.clear();
  for(int i=0; i<plan.markers.size(); i++){
    this->myPlan_.push_back(plan.markers[i].pose);
  }
};
void safetyProtocol::subscribeOthersPlan(const visualization_msgs::MarkerArray plan){
  this->othersPlan_.clear();
  for(int i=0; i<plan.markers.size(); i++) othersPlan_.push_back(plan.markers[i].pose);
  // Push current other robot's pose to history
  if(this->othersHistory_.size()==0){
    this->othersHistory_.push_back(plan.markers[0].pose);
  }
  else{
    if(this->dist(this->othersHistory_[0].position, plan.markers[0].pose.position)>0.5){
      this->othersHistory_.push_back(plan.markers[0].pose);
    }
  }
};
float safetyProtocol::dist(geometry_msgs::Point my_plan_pt, geometry_msgs::Point others_plan_pt){
  return hypot(my_plan_pt.x - others_plan_pt.x, my_plan_pt.y - others_plan_pt.y);
};
bool safetyProtocol::distanceBaseAlarm(float warningDistance = 0.6){
  bool danger = false;
  if(myPlan_.size() == 0 || othersPlan_.size() == 0) return danger;

  for(int i=0; i<myPlan_.size()||i<othersPlan_.size(); i++){
    if(i<myPlan_.size()){
      if(dist(myPlan_[i].position, othersPlan_[0].position) < warningDistance){
        ROS_INFO_STREAM("Roberto--->Marvin");
        ROS_INFO_STREAM("my pose : ("<<myPlan_[i].position.x<<", "<<myPlan_[i].position.y<<")");
        ROS_INFO_STREAM("others pose : ("<<othersPlan_[0].position.x<<", "<<othersPlan_[0].position.y<<")");
        danger = true;
        return danger;
      }
    }
    if(i<othersPlan_.size()){
      if(dist(myPlan_[0].position, othersPlan_[i].position) < warningDistance){
        ROS_INFO_STREAM("Roberto<---Marvin");
        ROS_INFO_STREAM("my pose : ("<<myPlan_[0].position.x<<", "<<myPlan_[0].position.y<<")");
        ROS_INFO_STREAM("others pose : ("<<othersPlan_[i].position.x<<", "<<othersPlan_[i].position.y<<")");
        danger = true;
        return danger;
      }
    }
    if(i<myPlan_.size() && i<othersPlan_.size()){
      if(dist(myPlan_[i].position, othersPlan_[i].position) < warningDistance){
        ROS_INFO_STREAM("Roberto-><-Marvin");
        ROS_INFO_STREAM("my pose : ("<<myPlan_[i].position.x<<", "<<myPlan_[i].position.y<<")");
        ROS_INFO_STREAM("others pose : ("<<othersPlan_[i].position.x<<", "<<othersPlan_[i].position.y<<")");
        danger = true;
        return danger;
      }
    }
  }

  return danger;
};
void safetyProtocol::stopASPPlan(){
  // cancel current ASP planner
  this->client_->cancelGoal();
  // Manually stop the move base.
  actionlib_msgs::GoalID stop_msg;
  stop_msg.id = "";
  stop_rate->sleep();
  stop.publish(stop_msg);
};
float safetyProtocol::distance(geometry_msgs::Point start_pt, geometry_msgs::Point goal_pt){
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
      if(DEBUG) ROS_INFO_STREAM("No plan available. Return Euclidian distance");
      distance = dist(start_pt, goal_pt);
    }
  }
  else{
    ROS_INFO_STREAM("Planning failed!");
  }
  return distance;
}
string safetyProtocol::findSafeZone(){
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
  ROS_INFO_STREAM("nearest Safe Zone : "<<safeZone);
  return safeZone;
};
void safetyProtocol::resume(string location){
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
void safetyProtocol::goToSafeZone(string safeZone){
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
/*
bool safetyProtocol::waitUntilSafe(){
  // return true if it is safe, otherwise return false;
  bool safe = false;
  // if robots are 5 meters away
  vector<geometry_msgs::Pose>::iterator my_iter = myPlan_.begin();
  vector<geometry_msgs::Pose>::iterator others_iter = othersPlan_.end();

  float distance_between_robot = dist(my_iter->position, (--others_iter)->position);
  ////ROS_INFO_STREAM("DEBUG : "<<distance_between_robot);
  ////ROS_INFO_STREAM("DEBUG : "<<othersPlan_.size()<<" - other robot's plan size");
  //ROS_INFO_STREAM("waitUntilSafe init");
  if(distance_between_robot > 8.0f){
    ROS_INFO_STREAM("Robots are 8.0 M away!");
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
    ROS_INFO_STREAM("Other robot is definately planning to move away!");
    safe = true;
  }

  return safe;
};
*/
bool safetyProtocol::waitUntilSafe(){
  // return true if it is safe, otherwise return false;
  bool safe = false;
  // if robots are 5 meters away
  vector<geometry_msgs::Pose>::iterator my_iter = myPlan_.begin();
  vector<geometry_msgs::Pose>::iterator others_iter = othersPlan_.end();

  // if distance between me and future plan of other robot is 8.0m away, resume.
  float distance_between_robot = dist(my_iter->position, (--others_iter)->position);
  if(distance_between_robot > 8.0f){
    ROS_INFO_STREAM("Robots are 8.0 M away!");
    safe = true;
    return safe;
  }
  // If other robot is stopped and planning for next location, it is not safe.
  if(othersPlan_.size() == 2){
    return safe;
  }
  // check if other robot is moving away.
  bool isMovingAway = false;

  int idx = 0;
  float prev_dist = dist(my_iter->position, this->othersHistory_[idx].position);
  float curr_dist;
  idx++;

  for(; idx < buffer_size; idx++){
    // write distance
    curr_dist = dist(my_iter->position, this->othersHistory_[idx].position);
    if(curr_dist>prev_dist){
      isMovingAway = true;
      break;
    }
  }
  //ROS_INFO_STREAM("waitUntilSafe far away finished");
  if(isMovingAway == true){
    ROS_INFO_STREAM("Other robot is definately planning to move away!");
    safe = true;
  }

  return safe;
};

void safetyProtocol::run(string location){
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
    if(DEBUG) ROS_INFO_STREAM("DEBUG 1");
    // Decide when to resume robot.
    //double stime = clock();
    while(ros::ok()){
      //if((clock()-stime)/CLOCKS_PER_SEC > 600) break;
      // Handel ABORTED from bwi logical action
      if(DEBUG) ROS_INFO_STREAM("DEBUE 1-1");
      if(this->lac_->getState() == actionlib::SimpleClientGoalState::ABORTED){
        ROS_INFO_STREAM("Execute Logical Goal failed.");
        ROS_INFO_STREAM("Re-executing.");
        goToSafeZone(safeZone);
      }
      if(DEBUG) ROS_INFO_STREAM("DEBUE 1-2");
      //wait until it is safe.
      if(waitUntilSafe()) break;
      stop_rate->sleep();
    }
    if(DEBUG) ROS_INFO_STREAM("DEBUG 2");
    // resume to original goal
    if(this->lac_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
      this->lac_->cancelGoal();
    }
    if(DEBUG) ROS_INFO_STREAM("DEBUG 3");
    resume(location);
    while(ros::ok()){
      if(this->client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED) break;
      ROS_INFO_STREAM("waiting for robot to resume");
      this->stop_rate->sleep();
    }
    if(DEBUG) ROS_INFO_STREAM("DEBUG 4");
  }
  for(int i=0; i<10; i++) this->stop_rate->sleep();
  return;
};

void safetyProtocol::goTo(string location){
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
int main(int argc, char** argv){
  ros::init(argc, argv, "sample_visit_door_safely");

  safetyProtocol sp(ROBOT_NAME);

  // initialize door list
  vector<string> doors;
  doors.push_back("d3_414b1");
  doors.push_back("d3_414b2");
  doors.push_back("d3_414a1");
  doors.push_back("d3_414a2");
  doors.push_back("d3_418");
  int door = 0;

  sp.goTo(doors.at(door));

  while(ros::ok()){
    sp.run(doors.at(door));
    if(DEBUG) ROS_INFO_STREAM("main DEBUG 1");
    if(sp.isClientArrive()){
      if(REVERSE) door = (door?door:doors.size())-1;
      else        door = (door+1)%doors.size();
      //ROS_INFO_STREAM("-1 < " << door << " < " << doors.size());

      if(DEBUG) ROS_INFO_STREAM("main DEBUG 2-1");
      sp.goTo(doors.at(door));
    }
    if(DEBUG) ROS_INFO_STREAM("main DEBUG 2-2");
  }
}
