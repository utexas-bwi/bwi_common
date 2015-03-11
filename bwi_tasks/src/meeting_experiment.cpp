
#include "bwi_kr_execution/ExecutePlanAction.h"

#include <actionlib/client/simple_action_client.h>

#include <ros/ros.h>

typedef actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction> Client;

using namespace std;

int main(int argc, char**argv) {
  ros::init(argc, argv, "meeting_experiment");
  ros::NodeHandle n;
  
  ros::NodeHandle privateNode("~");
  string meeting;
  privateNode.param<string>("meeting",meeting,"bwi_meeting");
  
  ROS_INFO_STREAM("getting ready for meeting " << meeting);
  
  Client client("/action_executor/execute_plan", true);
  client.waitForServer();
  
  bwi_kr_execution::ExecutePlanGoal goal;
  
  bwi_kr_execution::AspRule rule;
  bwi_kr_execution::AspFluent fluent;
  fluent.name = "not allinmeeting";
  
  fluent.variables.push_back(meeting);
 
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