#ifndef CHICKENSAFETY_H
#define CHICKENSAFETY_H

#include <actionlib/client/simple_action_client.h>
#include <plan_execution/ExecutePlanAction.h>
#include <bwi_msgs/LogicalActionAction.h>

#include <nav_msgs/GetPlan.h>

#include <visualization_msgs/MarkerArray.h>

#include <ros/ros.h>

#include <bwi_planning_common/structures.h>

typedef actionlib::SimpleActionClient<plan_execution::ExecutePlanAction> ASPClient;
typedef actionlib::SimpleActionClient<bwi_msgs::LogicalActionAction> LAClient;

using namespace std;

class ChickenSafetyProtocol{

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
  // private functions
  void subscribeMyPlan(const visualization_msgs::MarkerArray plan);
  void subscribeOthersPlan(const visualization_msgs::MarkerArray plan);
  float dist(geometry_msgs::Point my_plan_pt, geometry_msgs::Point others_plan_pt);
  float distance(const geometry_msgs::Point start_pose, const geometry_msgs::Point goal_pose);


public:
  void stopASPPlan();
  void resume(string location);
  void run(string location);
  void goTo(string location);

  ChickenSafetyProtocol(string ROBOT_NAME, string HOST_NAME);
  bool distanceBaseAlarm(float warningDistance);
  string findSafeZone();
  void goToSafeZone(string location);
  bool waitUntilSafe();

  bool isClientArrive(){
    if(this->client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) return true;
    return false;
  }
};
#endif
