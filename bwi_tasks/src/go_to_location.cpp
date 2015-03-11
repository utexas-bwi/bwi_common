
#include "bwi_kr_execution/ExecutePlanAction.h"

#include <actionlib/client/simple_action_client.h>

#include <ros/ros.h>

typedef actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction> Client;

using namespace std;

int main(int argc, char**argv) {
  ros::init(argc, argv, "go_to_location");
  ros::NodeHandle n;
  
  ros::NodeHandle privateNode("~");
  string location;
  privateNode.param<string>("location",location,"l3_414b");
  
  ROS_INFO_STREAM("going to " << location);
  
  Client client("action_executor/execute_plan", true);
  client.waitForServer();
  
  bwi_kr_execution::ExecutePlanGoal goal;
  
  bwi_kr_execution::AspRule rule;
  bwi_kr_execution::AspFluent fluent;
  fluent.name = "not at";
  
  fluent.variables.push_back(location);
 
  rule.body.push_back(fluent);
  goal.aspGoal.push_back(rule);
  
  ROS_INFO("sending goal");
  client.sendGoalAndWait(goal);
  
  if (client.getState() == actionlib::SimpleClientGoalState::ABORTED) {
    ROS_INFO("Aborted");
  }
  else if (client.getState() == actionlib::SimpleClientGoalState::PREEMPTED) {
    ROS_INFO("Preempted");
  }
  
  else if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Succeeded!");
  }
  else
     ROS_INFO("Terminated");
    
  return 0;
}
