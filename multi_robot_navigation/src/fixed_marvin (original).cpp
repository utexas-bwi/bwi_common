
#include "plan_execution/ExecutePlanAction.h"

#include <actionlib/client/simple_action_client.h>

#include <ros/ros.h>

typedef actionlib::SimpleActionClient<plan_execution::ExecutePlanAction> Client;

using namespace std;

void executePlan(Client* client, string location){
  ROS_INFO_STREAM("Going to "<<location);
  plan_execution::ExecutePlanGoal goal;
  plan_execution::AspRule rule;
  plan_execution::AspFluent fluent;

  fluent.name = "not facing";
  fluent.variables.push_back(location);
  rule.body.push_back(fluent);
  goal.aspGoal.push_back(rule);

  client->sendGoal(goal);
}
int main(int argc, char** argv){
  ros::init(argc, argv, "fixed_marvin");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);

  Client *client;
  client = new Client("/marvin/action_executor/execute_plan", true);
  client->waitForServer();

  // initialize door list
  vector<string> doors;
  doors.push_back("d3_414b1");
  doors.push_back("d3_414b2");
  doors.push_back("d3_414a1");
  doors.push_back("d3_414a2");
  doors.push_back("d3_418");
  int door = 0;

  executePlan(client, doors.at(door));

  while(ros::ok()){
    if(client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      door = (door+1)%doors.size();
      executePlan(client, doors.at(door));
    }
    else if(client->getState() == actionlib::SimpleClientGoalState::ABORTED){
      ROS_INFO_STREAM("Aborting. Process to next door.");
      executePlan(client, doors.at(door+1));
    }
    loop_rate.sleep();
  }
}
