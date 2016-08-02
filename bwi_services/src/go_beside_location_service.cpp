#include "bwi_kr_execution/ExecutePlanAction.h"
#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
#include "bwi_services/GoBesideLocation.h"

typedef actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction> Client;

Client* client;

using namespace std;

bool go_beside_location(bwi_services::GoBesideLocation::Request &req,
    bwi_services::GoBesideLocation::Response &res) {
  ROS_INFO("requesting goBesideLocation: %s", req.location.c_str());

  ROS_INFO("waiting for server");
  client->waitForServer();
  
  ROS_INFO("creating goal");
  bwi_kr_execution::ExecutePlanGoal goal;
  
  bwi_kr_execution::AspRule rule;
  bwi_kr_execution::AspFluent fluent;
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

int main(int argc, char**argv) {
  ros::init(argc, argv, "go_beside_location_service_node");
  ros::NodeHandle n;

  client = new Client("/action_executor/execute_plan", true);

  ros::ServiceServer service = n.advertiseService("/bwi_services/go_beside_location", go_beside_location);
  ROS_INFO("GoBesideLocation Service Started");

  ros::spin();
  ROS_INFO("Done spinning");
  return 0;
}
