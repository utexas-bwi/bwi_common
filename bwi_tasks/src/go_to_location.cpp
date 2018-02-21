
#include "plan_execution/ExecutePlanAction.h"

#include <actionlib/client/simple_action_client.h>

#include <ros/ros.h>

typedef actionlib::SimpleActionClient<plan_execution::ExecutePlanAction> Client;

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
  
  plan_execution::ExecutePlanGoal goal;
  
  plan_execution::AspRule rule;
  plan_execution::AspFluent fluent;
  fluent.name = "not at";
  
  fluent.variables.push_back(location);
 
  rule.body.push_back(fluent);
  goal.aspGoal.push_back(rule);
  
  ROS_INFO("sending goal");
  client.sendGoal(goal);
  
  ros::Rate wait_rate(10);
  while(ros::ok() && !client.getState().isDone())
    wait_rate.sleep();
    
  if (!client.getState().isDone()) {
    ROS_INFO("Canceling goal");
    client.cancelGoal();
    //and wait for canceling confirmation
    for(int i = 0; !client.getState().isDone() && i<50; ++i)
      wait_rate.sleep();
  }
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
