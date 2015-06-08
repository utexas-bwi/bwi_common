
#include "bwi_kr_execution/ExecutePlanAction.h"

#include <actionlib/client/simple_action_client.h>
#include <bwi_msgs/LogicalNavigationAction.h>

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

  Client client("action_executor/execute_plan", true);
  client.waitForServer();


    ROS_INFO_STREAM("Looking for " << person);

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
    
    bwi_kr_execution::AspRule flag_rule;
    bwi_kr_execution::AspFluent find_person_flag;
    find_person_flag.name = "findPersonTask";
    flag_rule.head.push_back(find_person_flag);

    
    goal.aspGoal.push_back(rule);
    goal.aspGoal.push_back(flag_rule);


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
    
  client.cancelAllGoals();
  
  return 0;
}
