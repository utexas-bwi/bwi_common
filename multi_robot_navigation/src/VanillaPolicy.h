#ifndef BWI_SAFETY_POLICY_VANILLA_POLICY_H
#define BWI_SAFETY_POLICY_VANILLA_POLICY_H
// Modules for subscribing topics
#include <visualization_msgs/MarkerArray.h>
// Modules for movebase control
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
// Ros and Utility modules
#include <tf/transform_listener.h>
#include <ros/ros.h>

#include <nav_msgs/GetPlan.h>
#include "multi_robot_collision_avoidance/EvalWaypoint.h"

// Modules for yaml file reading
#include <boost/filesystem.hpp>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// Define basic functions for safety policy.
// Comment will be consists of Module: purpose
class VanillaPolicy{
private:
  std::string myName;
  // Detect Collision: Subscribe local plan
  ros::Subscriber subscribeMyLocalPlan_;
  void subscribeMyLocalPlan(const visualization_msgs::MarkerArray plan);
  ros::Subscriber subscribeOthersLocalPlan_;
  void subscribeOthersLocalPlan(const visualization_msgs::MarkerArray plan);
  // Resume: Subscribe my goal
  ros::Subscriber subscribeMyGoal_;
  void subscribeMyGoal(const move_base_msgs::MoveBaseActionGoal goal);

  ros::AsyncSpinner *spinner;
  // Detect Collision
  std::vector<bool> safetyStack;
  //
  geometry_msgs::Pose prev_pose;

  void loadParams();
  void initializeSubscribers();
  //Chicken==============

  std::map<std::string, geometry_msgs::Pose> parkingZones_;
  std::map<std::string, geometry_msgs::Pose>::iterator iter_;

  bool readObjectApproachFile(const std::string& filename,
    std::map<std::string, geometry_msgs::Pose>& object_approach_map);
  void readParkingZones();
  float dist2ParkingZone(geometry_msgs::Pose start_pose, geometry_msgs::Pose goal_pose);

  ros::ServiceClient makePlanService_;
  // OBEY =====================
  ros::ServiceClient obey_find;
protected:
  MoveBaseClient client_ac_;
  std::string frame_id_;
  geometry_msgs::Pose goal_pose_;

  std::string PLATFORM;
  bool DEBUG;
  int PRUDENCE;

  ros::NodeHandle nh_;
  ros::Rate rate;

  std::vector<geometry_msgs::Pose> myLocalPlan_;
  std::vector<geometry_msgs::Pose> othersLocalPlan_;
  geometry_msgs::Pose previous_goal;

  float dist(geometry_msgs::Point myPoint, geometry_msgs::Point othersPoint);

  bool toWaypoint;
  bool storeNewGoal;
public:
  VanillaPolicy(ros::NodeHandle* nodehandle, std::string robot_name);

  geometry_msgs::Pose coord2Pose(float x, float y, float yaw);
  geometry_msgs::Pose coord2Pose(std::vector<float> coord);
  void setGoal(float x, float y, float yaw);

  void move(){ move(this->goal_pose_); };
  void move(geometry_msgs::Pose goal_pose);
  bool detectCollision(float radius);
  void stop();
  geometry_msgs::Pose findWaypoint_chicken();
  geometry_msgs::Pose findWaypoint_obey();
  bool waitUntilSafe();
  bool isArrive();
  bool isAborted();
  void resume();

  void safety();
  void safety_chicken();
  void safety_obey();
// Added function
  bool is_other_arrive(std::vector<float> coord);
};
#endif
