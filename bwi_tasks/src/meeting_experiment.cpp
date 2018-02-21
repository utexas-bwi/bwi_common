
#include "plan_execution/ExecutePlanAction.h"

#include <actionlib/client/simple_action_client.h>

#include <ros/ros.h>

typedef actionlib::SimpleActionClient<plan_execution::ExecutePlanAction> Client;

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
  
  plan_execution::ExecutePlanGoal goal;
  
  plan_execution::AspRule rule;
  plan_execution::AspFluent fluent;
  fluent.name = "not allinmeeting";
  fluent.variables.push_back(meeting);
  rule.body.push_back(fluent);
  
  plan_execution::AspRule flag_rule;
  plan_execution::AspFluent find_person_flag;
  find_person_flag.name = "findPersonTask";
  flag_rule.head.push_back(find_person_flag);
  
  
  goal.aspGoal.push_back(rule);
  goal.aspGoal.push_back(flag_rule);
  
  
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