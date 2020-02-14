
#include "plan_execution/ExecutePlanAction.h"

#include <actionlib/client/simple_action_client.h>

#include <bwi_planning_common/structures.h>
#include <bwi_planning_common/utils.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <nav_msgs/GetPlan.h>

#include <ros/ros.h>

#include <string>
#include <iostream>
#include <fstream>

typedef actionlib::SimpleActionClient<plan_execution::ExecutePlanAction> Client;

using namespace std;

string ROBOT_NAME = "roberto/";
string HOST_NAME = "jinsoo";
bool DEBUG = true;
bool REVERSE = true; // if on, robot will turn clockwise.
string global_frame_id_ = ROBOT_NAME + "level_mux_map";

float tolerance_ = 1.5; // tolerance for safe interaction point
int warningDistance = 10;

class Visit_Door_safely {
public:
  Visit_Door_safely(ros::NodeHandle* nh);
  void run(bool REVERSE);
  void callback();
  void callback1(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose){
    my_odom = pose->pose;
  }
  void callback2(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose){
    others_odom = pose->pose;
  }
  vector<geometry_msgs::Point> getInteractionPoints(const geometry_msgs::Point &start_pt, const geometry_msgs::Point &goal_pt);
  geometry_msgs::Point getInteractionPoint(const geometry_msgs::PoseWithCovariance &my_odom, const geometry_msgs::PoseWithCovariance &other_odom);
  bool safeInteraction(const geometry_msgs::Point &interaction_pt, const geometry_msgs::Point &odom, const geometry_msgs::Point &other_odom);
  string getNearestParkingZone(const geometry_msgs::Point &ip, const geometry_msgs::Point &my_odom, const geometry_msgs::Point &other_odom);

  bool checkRobotsMovingCloser(vector<int> &distances){
    int size = distances.size();
    int i = size-35 > 0 ? size-35: 0;
    int count = 0;
    float min, max;
    for( ; i<size-1; i++){
      if(distances[i]>distances[i+1] && distances[i] - distances[i+1] < 3 && distances[i] < 12){
        count++;
        if(distances[i]>max && count == 1){
          max = distances[i];
        }
        else if(count==3 && (max-distances[i+1])>1){
          return false;
        }
      }
    }
    return true;
  }
  bool checkRobotsMovingAway(){
      int count = 0;
      float min = 1000;
      float max = 0;

      int history_size = distance_history.size();

      for(int i = (history_size-15 > 0 ? history_size-15:0); i<history_size-1; i++){
        if(distance_history[i] < distance_history[i+1]){
          count++;
          if(count==1){
            min = distance_history[i];
          }
          if(count>3 && (distance_history[i+1]-min) > 0.5){
            return true;
          }
        }
      }
      return false;
  }
  void stopRobot(Client *client){
    ros::Publisher pub;
    actionlib_msgs::GoalID msg;
    msg.id = "";
    stop.publish(msg);
    client->cancelGoal();
  }
  void resumeRobot(Client *client, int door){
    //Exception case
    if(parkedLocation.compare("p3_15") == 0 || parkedLocation.compare("p3_16") == 0){
      string tempLocation = "r3_16";
      ROS_INFO_STREAM("Going to temp loc: " << tempLocation);
      execute_plan_and_wait(tempLocation);
    }

    string location = doors.at(door);
    ROS_INFO_STREAM(ROBOT_NAME + " going to " + location);
    execute_plan(location);
  }
  void execute_plan(string location){
    plan_execution::ExecutePlanGoal goal;
    plan_execution::AspRule rule;
    plan_execution::AspFluent fluent;
    fluent.name = "not facing";

    fluent.variables.push_back(location);
    rule.body.push_back(fluent);
    goal.aspGoal.push_back(rule);
    client->sendGoal(goal);
  }
  void execute_plan_and_wait(string location){
    plan_execution::ExecutePlanGoal goal;
    plan_execution::AspRule rule;
    plan_execution::AspFluent fluent;
    fluent.name = "not facing";

    fluent.variables.push_back(location);
    rule.body.push_back(fluent);
    goal.aspGoal.push_back(rule);
    client->sendGoalAndWait(goal);
  }
  void goToParkingZone(string ParkingZone, Client *client);
protected:
  ros::NodeHandle* nh_;
  ros::Rate* loop_rate;

  Client *client;
  map<string, geometry_msgs::Pose> ParkingZones;

  int door;
  vector<string> doors;

  ros::ServiceClient make_plan;
  ros::Publisher stop;

  bool parked;
  int savedGoal;
  string parkedLocation;
  vector<string> notSafe;
  vector<float> distance_history;
  vector<int> robot_distances;

  ros::ServiceClient make_plan_client;

  geometry_msgs::PoseWithCovariance my_odom;
  geometry_msgs::PoseWithCovariance others_odom;
  ofstream myfile;
};

Visit_Door_safely::Visit_Door_safely(ros::NodeHandle* nh){
  if(DEBUG == true)
  {
    myfile.open("DEBUG_log.txt");
    if(!myfile.is_open())
    {
      ROS_INFO_STREAM("DEBUGging log file is not open. Please check ./DEBUG_log.txt file.");
      exit(1);
    }
  }

  nh_ = nh;
  loop_rate = new ros::Rate(10);
  parked = false;

  client = new Client("/" + ROBOT_NAME + "action_executor/execute_plan", true);
  stop = nh_->advertise<actionlib_msgs::GoalID>("/" + ROBOT_NAME + "move_base/cancel", 1000, true);

  notSafe.push_back("p3_15");
  notSafe.push_back("p3_16");

  // Declare data directory to pre-defined safe zones.
  string data_directory = "/home/" + HOST_NAME + "/catkin_ws/src/bwi_common/utexas_gdc/maps/simulation/3ne";
  string loc_file = bwi_planning_common::getObjectsFileLocationFromDataDirectory(data_directory);
  // Read all pre-defined safe zones from object.yaml file
  ROS_INFO_STREAM("Reading Parking Zone file: " + loc_file);
  bwi_planning_common::readObjectApproachFile(loc_file, ParkingZones);
  ROS_INFO_STREAM("ParkingZones size: " << ParkingZones.size());
  // Log all possible parking zones.
  map<string, geometry_msgs::Pose>::iterator iter;

  for(iter = ParkingZones.begin(); iter != ParkingZones.end(); iter++){
    if(iter->first.at(0) == 'r'){
      ParkingZones.erase(iter);
      --iter;
    }
    else{
      ROS_INFO_STREAM(iter->first);
    }
  }
};
geometry_msgs::Point Visit_Door_safely::getInteractionPoint(const geometry_msgs::PoseWithCovariance &my_odom, const geometry_msgs::PoseWithCovariance &other_odom){
  vector<geometry_msgs::Point> interaction_points = getInteractionPoints(my_odom.pose.position, other_odom.pose.position);
  int plan_size = interaction_points.size();
  /*ROS_INFO_STREAM(plan_size);
  ROS_INFO_STREAM("\n"<<interaction_points[0]<<"\n"<<interaction_points[plan_size/2]<<"\n"<<interaction_points[plan_size-1]);
*/
  return interaction_points[(plan_size/2)];
}
vector<geometry_msgs::Point> Visit_Door_safely::getInteractionPoints(const geometry_msgs::Point &start_pt, const geometry_msgs::Point &goal_pt){
  vector<geometry_msgs::Point> interaction_pts;
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

  interaction_pts.push_back(start.pose.position);

  if(!make_plan_client){
    ROS_INFO("GetPlan service connection failed");
  }
  make_plan_client = nh_->serviceClient<nav_msgs::GetPlan>(ROBOT_NAME + "move_base/NavfnROS/make_plan");
  make_plan_client.waitForExistence();

  float prev_x = interaction_pts.back().x;
  float prev_y = interaction_pts.back().y;

  if(make_plan_client.call(srv)) {
    if(srv.response.plan.poses.size() != 0){
      for(int i = 0 ; i<srv.response.plan.poses.size() ; i++){
        geometry_msgs::PoseStamped p = srv.response.plan.poses[i];
        if(abs(prev_x - p.pose.position.x) > 1 || abs(prev_y - p.pose.position.y)>1){
          interaction_pts.push_back(p.pose.position);
          prev_x = p.pose.position.x;
          prev_y = p.pose.position.y;
        }
      }
    }
    else{
      //ROS_INFO_STREAM("Empty Plan!");
    }
  }
  else{
    //ROS_INFO_STREAM("Planning failed!");
  }
  //ROS_INFO_STREAM(interaction_pts);
  return interaction_pts;
}
string Visit_Door_safely::getNearestParkingZone(const geometry_msgs::Point &ip, const geometry_msgs::Point &my_odom, const geometry_msgs::Point &other_odom){
  string nearestParkingZone;
  float min_distance = INT_MAX, distance;
  geometry_msgs::Point safe_zone;
  bool empty = true;
  map<string, geometry_msgs::Pose>::iterator iter;

  for(iter = ParkingZones.begin(); iter != ParkingZones.end(); iter++){
    ROS_INFO_STREAM(iter->first);
    safe_zone.x = iter->second.position.x;
    safe_zone.y = iter->second.position.y;
    //distance = getInteractionPoints(safe_zone, ip).size(); // CASE 1
    distance = getInteractionPoints(my_odom, safe_zone).size(); // CASE 2
    if(distance < min_distance && distance != 1){
/*//CASE 1
      int my_dist = getInteractionPoints(my_odom, safe_zone).size();
      int other_dist = getInteractionPoints(other_odom, safe_zone).size();
      ROS_INFO_STREAM(my_dist);
      ROS_INFO_STREAM(other_dist);
      if(my_dist < other_dist){
        min_distance = distance;
        nearestParkingZone = iter->first;
        empty=false;
      }*/
//CASE 2
      min_distance = distance;
      nearestParkingZone = iter->first;
      empty = false;
    }
  }
  if(empty){
    //ROS_INFO_STREAM("No nearest Zone");
    nearestParkingZone = "empty";
  }
  ROS_INFO_STREAM("nearestZone: " + nearestParkingZone);
  return nearestParkingZone;
}
void Visit_Door_safely::goToParkingZone(string ParkingZone, Client *client){
  ROS_INFO_STREAM(ROBOT_NAME + " going to " + ParkingZone);
  execute_plan_and_wait(ParkingZone);
  //execute_plan_and_wait(ParkingZone); // TEST 1

  if (client->getState() == actionlib::SimpleClientGoalState::ABORTED) {ROS_INFO("Aborted");}
  else if (client->getState() == actionlib::SimpleClientGoalState::PREEMPTED) {ROS_INFO("Preempted");}
  else if (client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    stopRobot(client);
    parkedLocation = ParkingZone;
    ROS_INFO_STREAM("Succefully parked at" + parkedLocation);
  }
  else{ROS_INFO("Terminated");}
}
bool Visit_Door_safely::safeInteraction(const geometry_msgs::Point &interaction_pt, const geometry_msgs::Point &odom, const geometry_msgs::Point &other_odom){
  geometry_msgs::Point safe_zone;

  int distance = getInteractionPoints(odom, other_odom).size();
  float odom_distance = sqrt(pow(odom.x-other_odom.x, 2) + pow(odom.y-other_odom.y,2));
  if(odom_distance>0.1 && odom_distance < warningDistance){
    distance_history.push_back(odom_distance);
  }
  robot_distances.push_back(distance);

  map<string, geometry_msgs::Pose>::iterator iter;
  /*
  for(iter = ParkingZones.begin(); iter != ParkingZones.end(); iter++){
    safe_zone.x = iter->second.position.x;
    safe_zone.y = iter->second.position.y;

    if(abs(safe_zone.x - interaction_pt.x)<tolerance_ && abs(safe_zone.y - interaction_pt.y)<tolerance_ && find(notSafe.begin(), notSafe.end(), iter->first) == notSafe.end()){
      //ROS_INFO_STREAM("interaction occurs in safe zone");
      return true;
    }
  }
*/
  if(checkRobotsMovingCloser(robot_distances)){
    //ROS_INFO_STREAM("robots moving away from each other");
    return true;
  }
  //ROS_INFO_STREAM("robots moving towards each other");
  return false;
}
void Visit_Door_safely::callback(){
// Make a code without parked function first.
  if(client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED && !parked){
    if(REVERSE){
      door = (door - 1);
      if(door < 0){
        door = door + (int)doors.size();
      }
    }
    else{
      door = (door + 1)%((int)doors.size());
    }
    string location = doors.at(door);
    ROS_INFO_STREAM(ROBOT_NAME + " going to " + location);
    execute_plan(location);
  }

  geometry_msgs::Point interaction_point = getInteractionPoint(my_odom, others_odom);

  if(DEBUG){
    //ROS_INFO_STREAM("interaction point : "<<interaction_point.x<<" "<<interaction_point.y);
  }

  bool safe = safeInteraction(interaction_point, my_odom.pose.position, others_odom.pose.position);
//  ROS_INFO_STREAM(interaction_point << "\n" << my_odom.pose.position << "\n" << others_odom.pose.position);
//  ROS_INFO_STREAM("safe : " << safe << "  parked" << parked);

//  ROS_INFO_STREAM("Distance : " << distance_history.size());
  if(parked){
    if(checkRobotsMovingAway()){
      ROS_INFO("resume to original goal");
      resumeRobot(client, savedGoal);
      parked = false;
    }
  }
  else if(!safe){
    string nearSafeZone = getNearestParkingZone(interaction_point, my_odom.pose.position, others_odom.pose.position);

    if(nearSafeZone.compare("empty") == 0){
      nearSafeZone = getNearestParkingZone(my_odom.pose.position, my_odom.pose.position, others_odom.pose.position);
    }
    //ROS_INFO_STREAM("Roberto going to " + nearSafeZone);
    if(nearSafeZone.compare("empty") && DEBUG){
      myfile<<"my position:\n";
      myfile<<my_odom.pose.position << "\t" << others_odom.pose.position;
    }
    distance_history.clear();
    robot_distances.clear();

    savedGoal = door;
    goToParkingZone(nearSafeZone, client);
    parked = true;
    //ROS_INFO("Parking Roberto to position");
  }
}
void Visit_Door_safely::run(bool REVERSE){
  ROS_INFO_STREAM(ROBOT_NAME + " will explore doors " + (REVERSE ? "" : "Counter") + " Clockwise.");
  // initialize door lists.
  doors.push_back("d3_414a2");
  doors.push_back("d3_414b1");
  doors.push_back("d3_414b2");
  doors.push_back("d3_414a1");
  door = (REVERSE ? (int)doors.size()-1:0);

  client->waitForServer();

  string location = doors.at(door);

  ROS_INFO_STREAM(ROBOT_NAME + " going to " << location);
  execute_plan(location);

  while(ros::ok()){
    loop_rate->sleep();
    //ROS_INFO_STREAM("Roberto odom : "<<my_odom.pose.position.x<<" "<<my_odom.pose.position.y);
    //ROS_INFO_STREAM("Marvin odom : "<<others_odom.pose.position.x<<" "<<others_odom.pose.position.y);
    callback();
  }
  return;
}
int main(int argc, char**argv) {
  ros::init(argc, argv, "between_doors_cleaver_roberto");
  ros::NodeHandle nh;

  Visit_Door_safely vds(&nh);

  ros::Subscriber sub1 = nh.subscribe("roberto/amcl_pose", 1000, &Visit_Door_safely::callback1, &vds);
  ros::Subscriber sub2 = nh.subscribe("marvin/amcl_pose", 1000, &Visit_Door_safely::callback2, &vds);

  ros::AsyncSpinner spinner(0);
  spinner.start();

  vds.run(REVERSE);

  return 0;
}
