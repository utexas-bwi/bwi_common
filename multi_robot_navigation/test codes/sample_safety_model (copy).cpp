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

typedef actionlib::SimpleActionClient<plan_execution::ExecutePlanAction> ASPClient;
typedef actionlib::SimpleActionClient<bwi_msgs::LogicalActionAction> LAClient;

using namespace std;

bool REVERSE = true;
string ROBOT_NAME = "/roberto";
string HOST_NAME  = "/jinsoo";

class safetyProtocol{
protected:
  ASPClient *client_;
  LAClient *lac_;
  ros::Publisher stop;
  ros::Rate *stop_rate;

  ros::NodeHandle nh_;
  vector<geometry_msgs::Pose> myPlan_;
  vector<geometry_msgs::Pose> othersPlan_;
  ros::Subscriber subscribeMyPlan_;
  ros::Subscriber subscribeOthersPlan_;
  ros::AsyncSpinner *spinner;

  map<string, geometry_msgs::Pose> parkingZones_;
  ros::ServiceClient makePlanService;
  string global_frame_id_;
public:
  safetyProtocol(ASPClient *client, string ROBOT_NAME);
  void subscribeMyPlan(const visualization_msgs::MarkerArray plan);
  void subscribeOthersPlan(const visualization_msgs::MarkerArray plan);
  float dist(geometry_msgs::Point my_plan_pt, geometry_msgs::Point others_plan_pt);
  bool distanceBaseAlarm(float warningDistance);
  void stopASPPlan();
  void goToSafeZone(string location);
  float distance(const geometry_msgs::Point start_pose, const geometry_msgs::Point goal_pose);
  string findSafeZone();
  bool waitUntilSafe();
  void resume(string location);
  void run(string location);
};
safetyProtocol::safetyProtocol(ASPClient *client, string ROBOT_NAME){
  this->client_ = client;
  this->lac_ = new LAClient(ROBOT_NAME + "/execute_logical_goal", true);
  stop = nh_.advertise<actionlib_msgs::GoalID>(ROBOT_NAME + "/move_base/cancel", 1, true);
  this->makePlanService = nh_.serviceClient<nav_msgs::GetPlan>(ROBOT_NAME + "/move_base/NavfnROS/make_plan");
  stop_rate = new ros::Rate(1000);

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
    //ROS_INFO("My Plan #%d: (%2.3f, %2.3f)", i+1, plan.markers[i].pose.position.x, plan.markers[i].pose.position.y);
  }
};
void safetyProtocol::subscribeOthersPlan(const visualization_msgs::MarkerArray plan){
  this->othersPlan_.clear();
  for(int i=0; i<plan.markers.size(); i++) othersPlan_.push_back(plan.markers[i].pose);
};
float safetyProtocol::dist(geometry_msgs::Point my_plan_pt, geometry_msgs::Point others_plan_pt){
  return hypot(my_plan_pt.x - others_plan_pt.x, my_plan_pt.y - others_plan_pt.y);
};
bool safetyProtocol::distanceBaseAlarm(float warningDistance = 0.6){
  bool danger = false;

  vector<geometry_msgs::Pose>::iterator my_iter = myPlan_.begin();
  vector<geometry_msgs::Pose>::iterator others_iter = othersPlan_.begin();
  // If the client begin running, smallest plan size will be 2.
  if(myPlan_.size() == 0 || othersPlan_.size() == 0){
    return danger;
  }
  // robot1---------><---------robot2
  while(ros::ok()){
    if(dist(my_iter->position, others_iter->position) < warningDistance){
      ROS_INFO_STREAM("Roberto-><-Marvin");
      danger = true;
      return danger;
    }
    ++my_iter;
    ++others_iter;
    if(my_iter == myPlan_.end() || others_iter == othersPlan_.end()) break;
  }
  // robot1--------->robot2
  my_iter = myPlan_.begin();
  others_iter = othersPlan_.begin();
  while(ros::ok()){
    if(dist(my_iter->position, others_iter->position) < warningDistance){
      ROS_INFO_STREAM("Roberto-->Marvin");
      danger = true;
      return danger;
    }
    ++my_iter;
    if(my_iter == myPlan_.end()) break;
  }
  // robot1<---------robot2
  my_iter = myPlan_.begin();
  while(ros::ok()){
    if(dist(my_iter->position, others_iter->position) < warningDistance){
      ROS_INFO_STREAM("Roberto<--Marvin");
      danger = true;
      return danger;
    }
    ++others_iter;
    if(others_iter == othersPlan_.end()) break;
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
  /*ROS_INFO_STREAM(start_pt<<"\n"<<goal_pt);*/
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
      ROS_INFO_STREAM("No plan available. Return Euclidian distance");
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
bool safetyProtocol::waitUntilSafe(){
  // return true if it is safe, otherwise return false;
  bool safe = false;
  // if robots are 5 meters away
  vector<geometry_msgs::Pose>::iterator my_iter = myPlan_.begin();
  vector<geometry_msgs::Pose>::iterator others_iter = othersPlan_.end();

  float distance_between_robot = dist(my_iter->position, (--others_iter)->position);
  //ROS_INFO_STREAM("DEBUG : "<<distance_between_robot);
  //ROS_INFO_STREAM("DEBUG : "<<othersPlan_.size()<<" - other robot's plan size");
  ROS_INFO_STREAM("waitUntilSafe init");
  if(distance_between_robot > 5.0f){
    ROS_INFO_STREAM("Robots are 5.0 M away!");
    //ROS_INFO_STREAM("distance : " << distance(myPlan_.begin()->position, othersPlan_.begin()->position));
    safe = true;
    return safe;
  }
  ROS_INFO_STREAM("waitUntilSafe distance between robots");
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
  ROS_INFO_STREAM("waitUntilSafe far away init");
  ROS_INFO_STREAM("Other plan size: "<<othersPlan_.size());
  int index = 1;
  for(; others_iter < othersPlan_.end(); others_iter++){
    // write distance
    ROS_INFO_STREAM("index : "<<index);
    index = index+1;
    curr_dist = dist(my_iter->position, others_iter->position);
    if(curr_dist<prev_dist){
      isMovingAway = false;
      break;
    }
  }
  ROS_INFO_STREAM("waitUntilSafe far away finished");
  if(isMovingAway == true){
    ROS_INFO_STREAM("Other robot is definately planning to move away!");
    safe = true;
  }

  return safe;
};
void safetyProtocol::run(string location){
  //ROS_INFO_STREAM("Safety Protocol");
  bool danger = distanceBaseAlarm();
  if(danger == true){
    //stop plan
    ROS_INFO_STREAM("Stop plan");
    stopASPPlan();
    // find near safezone
    string safeZone;
    safeZone = findSafeZone();
    // move to safe zone
    ROS_INFO_STREAM("Going to safe zone");
    goToSafeZone(safeZone);
    ROS_INFO_STREAM("DEBUG 1");
    // Decide when to resume robot.
    while(ros::ok()){
      // Handel ABORTED from bwi logical action
      ROS_INFO_STREAM("DEBUE 1-1");
      if(this->lac_->getState() == actionlib::SimpleClientGoalState::ABORTED){
        ROS_INFO_STREAM("Execute Logical Goal failed.");
        ROS_INFO_STREAM("Re-executing.");
        goToSafeZone(safeZone);
      }
      ROS_INFO_STREAM("DEBUE 1-2");
      //wait until it is safe.
      if(waitUntilSafe()) break;
      stop_rate->sleep();
    }
    ROS_INFO_STREAM("DEBUG 2");
    // resume to original goal
    if(this->lac_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
      this->lac_->cancelGoal();
    }
    ROS_INFO_STREAM("DEBUG 3");
    resume(location);
    ROS_INFO_STREAM("DEBUG 4");
    // wait until ASP planner start executing its plan.
    while(ros::ok()){
      if(this->client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
        break;
      }
      stop_rate->sleep();
    }
    ROS_INFO_STREAM("DEBUG 5");
  }
  return;
};

void executePlan(ASPClient* client, string location){
  ROS_INFO_STREAM("Going to "<<location);
  plan_execution::ExecutePlanGoal goal;
  plan_execution::AspRule rule;
  plan_execution::AspFluent fluent;

  fluent.name = "not facing";
  fluent.variables.push_back(location);
  rule.body.push_back(fluent);
  goal.aspGoal.push_back(rule);

  client->sendGoal(goal);
}
int main(int argc, char** argv){
  ros::init(argc, argv, "sample_visit_door_safely");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);

  ASPClient *client;
  client = new ASPClient(ROBOT_NAME + "/action_executor/execute_plan", true);
  client->waitForServer();

  safetyProtocol sp(client, ROBOT_NAME);

  // initialize door list
  vector<string> doors;
  doors.push_back("d3_414b1");
  doors.push_back("d3_414b2");
  doors.push_back("d3_414a1");
  doors.push_back("d3_414a2");
  doors.push_back("d3_418");
  int door = 0;

  executePlan(client, doors.at(door));

  while(ros::ok()){
    sp.run(doors.at(door));
    if(client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      if(REVERSE) door = (door?door:doors.size())-1;
      else        door = (door+1)%doors.size();
      ROS_INFO_STREAM("-1 < " << door << " < " << doors.size());
      executePlan(client, doors.at(door));
    }
    loop_rate.sleep();
  }
}
