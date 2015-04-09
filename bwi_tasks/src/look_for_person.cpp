
#include "bwi_kr_execution/ExecutePlanAction.h"

#include <actionlib/client/simple_action_client.h>

#include <std_srvs/Empty.h>
#include <ros/ros.h>

typedef actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction> Client;

using namespace std;

int main(int argc, char**argv) {
  ros::init(argc, argv, "look_for_person");
  ros::NodeHandle n;
  
  ros::NodeHandle privateNode("~");
  string person;
  privateNode.param<string>("person",person,"peter");
  
  ROS_INFO_STREAM("looking for " << person);
  
  Client client("action_executor/execute_plan", true);
  client.waitForServer();
  
  bwi_kr_execution::ExecutePlanGoal goal;
  
  bwi_kr_execution::AspRule rule;
  bwi_kr_execution::AspFluent fluent;
  fluent.name = "not ingdc";
  fluent.variables.push_back(person);
 
  rule.body.push_back(fluent);
  

  bwi_kr_execution::AspFluent fluent_not;
  fluent_not.name = "not -ingdc";
  fluent_not.variables.push_back(person);
  
  rule.body.push_back(fluent_not);
  
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
  
  ROS_INFO("Resetting the knowledge base (forgetting everything)");
  ros::ServiceClient forget_everything = n.serviceClient<std_srvs::Empty>("reset_state");
  
  std_srvs::Empty nothingness;
  forget_everything.call(nothingness);
  
    
  return 0;
}
