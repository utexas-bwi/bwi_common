#include "plan_execution/ExecutePlanAction.h"
#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
#include "bwi_virtour/GoToLocation.h"
#include "bwi_virtour/GoBesideLocation.h"
#include "bwi_virtour/Authenticate.h"
#include "bwi_virtour/Rotate.h"

typedef actionlib::SimpleActionClient<plan_execution::ExecutePlanAction> Client;

Client* client;

using namespace std;

bool goBesideLocation(bwi_virtour::GoBesideLocation::Request &req,
    bwi_virtour::GoBesideLocation::Response &res) {
  ROS_INFO("requesting goBesideLocation: %s", req.location.c_str());

  ROS_INFO("authenticating user: %s", req.user.c_str());
  bwi_virtour::Authenticate::Request auth_req;
  auth_req.user = req.user;
  bwi_virtour::Authenticate::Response auth_res;

  if (ros::service::call("/tour_manager/authenticate", auth_req, auth_res)) {
    if (auth_res.result < 0) {
      res.result = -5;
      ROS_INFO("Authentication failed!");
      return true;
    }
  }

  ROS_INFO("waiting for server");
  client->waitForServer();
  
  ROS_INFO("creating goal");
  plan_execution::ExecutePlanGoal goal;
  
  plan_execution::AspRule rule;
  plan_execution::AspFluent fluent;
  fluent.name = "not beside";
  
  fluent.variables.push_back(req.location);
 
  rule.body.push_back(fluent);
  goal.aspGoal.push_back(rule);
  
  ROS_INFO("sending goal");
  client->sendGoalAndWait(goal);
  
  if (client->getState() == actionlib::SimpleClientGoalState::ABORTED) {
    ROS_INFO("Aborted");
    res.result = -3;
  } else if (client->getState() == actionlib::SimpleClientGoalState::PREEMPTED) {
    ROS_INFO("Preempted");
    res.result = -2;
  } else if (client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Succeeded!");
    res.result = 1;
  } else {
     ROS_INFO("Terminated");
     res.result = -1;
  }
    
  ROS_INFO("sending back response: [%ld]", (long int)res.result);
  return true;
}

bool goToLocation(bwi_virtour::GoToLocation::Request &req,
    bwi_virtour::GoToLocation::Response &res) {
  ROS_INFO("requesting goToLocation: %s", req.location.c_str());

  ROS_INFO("authenticating user: %s", req.user.c_str());
  bwi_virtour::Authenticate::Request auth_req;
  auth_req.user = req.user;
  bwi_virtour::Authenticate::Response auth_res;

  if (ros::service::call("/tour_manager/authenticate", auth_req, auth_res)) {
    if (auth_res.result < 0) {
      res.result = -5;
      ROS_INFO("Authentication failed!");
      return true;
    }
  }

  ROS_INFO("waiting for server");
  client->waitForServer();
  
  ROS_INFO("creating goal");
  plan_execution::ExecutePlanGoal goal;
  
  plan_execution::AspRule rule;
  plan_execution::AspFluent fluent;
  fluent.name = "not at";
  
  fluent.variables.push_back(req.location);
 
  rule.body.push_back(fluent);
  goal.aspGoal.push_back(rule);
  
  ROS_INFO("sending goal");
  client->sendGoalAndWait(goal);
  
  if (client->getState() == actionlib::SimpleClientGoalState::ABORTED) {
    ROS_INFO("Aborted");
    res.result = -3;
  } else if (client->getState() == actionlib::SimpleClientGoalState::PREEMPTED) {
    ROS_INFO("Preempted");
    res.result = -2;
  } else if (client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Succeeded!");
    res.result = 1;
  } else {
     ROS_INFO("Terminated");
     res.result = -1;
  }
    
  ROS_INFO("sending back response: [%ld]", (long int)res.result);
  return true;
}
    

int main(int argc, char**argv) {
  ros::init(argc, argv, "go_to_location_service_node");
  ros::NodeHandle n;

  client = new Client("/action_executor/execute_plan", true);

  ros::ServiceServer service = n.advertiseService("go_to_location", goToLocation);
  ROS_INFO("GoToLocation Service Started");

  ros::ServiceServer service2 = n.advertiseService("go_beside_location", goBesideLocation);
  ROS_INFO("GoBesideLocation Service Started");

  ros::spin();
  ROS_INFO("Done spinning");
  return 0;
}
