
#include "plan_execution/ExecutePlanAction.h"

#include <actionlib/client/simple_action_client.h>

#include <ros/ros.h>

typedef actionlib::SimpleActionClient<plan_execution::ExecutePlanAction> Client;

using namespace std;

int main(int argc, char**argv) {
  ros::init(argc, argv, "between_doors");
  ros::NodeHandle n;

  ros::NodeHandle privateNode("~");

  Client client("marvin/action_executor/execute_plan", true);
  client.waitForServer();

  string location = "p3_15";
//  n.getParam("location", location);

  ROS_INFO_STREAM("going to " << location);

  plan_execution::ExecutePlanGoal goal;
  plan_execution::AspRule rule;
  plan_execution::AspFluent fluent;

  fluent.name = "not facing";
  fluent.variables.push_back(location);
  rule.body.push_back(fluent);
  goal.aspGoal.push_back(rule);

  ROS_INFO("sending goal");
  client.sendGoalAndWait(goal);

  return 0;
}
