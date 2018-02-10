
#include "plan_execution/ExecutePlanAction.h"

#include <actionlib/client/simple_action_client.h>

#include <ros/ros.h>

typedef actionlib::SimpleActionClient<plan_execution::ExecutePlanAction> Client;

using namespace std;

int main(int argc, char**argv) {
  ros::init(argc, argv, "back_and_forth");
  ros::NodeHandle n;

  ros::NodeHandle privateNode("~");
  string locationA;
  privateNode.param<string>("a",locationA,"l3_414b");

  string locationB;
  privateNode.param<string>("b",locationB,"l3_516");



  Client client("/action_executor/execute_plan", true);
  client.waitForServer();

  bool fromAtoB = true;

  while (ros::ok()) {

    string location = (fromAtoB)? locationB : locationA;

    fromAtoB = !fromAtoB;

    ROS_INFO_STREAM("going to " << location);

    plan_execution::ExecutePlanGoal goal;

    plan_execution::AspRule rule;
    plan_execution::AspFluent fluent;
    fluent.name = "not at";

    fluent.variables.push_back(location);

    rule.body.push_back(fluent);
    goal.aspGoal.push_back(rule);

    ROS_INFO("sending goal");
    client.sendGoalAndWait(goal);

    if (client.getState() == actionlib::SimpleClientGoalState::ABORTED) {
      ROS_INFO("Aborted");
    } else if (client.getState() == actionlib::SimpleClientGoalState::PREEMPTED) {
      ROS_INFO("Preempted");
    }

    else if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Succeeded!");
    } else
      ROS_INFO("Terminated");

  }

  return 0;
}