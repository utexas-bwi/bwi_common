
#include "bwi_kr_execution/ExecutePlanAction.h"

#include <actionlib/client/simple_action_client.h>
#include <bwi_msgs/LogicalNavigationAction.h>
#include <bwi_kr_execution/CurrentStateQuery.h>
#include <bwi_kr_execution/UpdateFluents.h>

#include <std_srvs/Empty.h>
#include <ros/ros.h>

typedef actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction> Client;

using namespace std;


struct TaskFluent {

  bool operator()(const bwi_kr_execution::AspFluent& fluent) const {

    return fluent.name == "inroom" ||
           fluent.name == "-inroom" ||
           fluent.name == "inoffice" ||
           fluent.name == "-inoffice" ||
           fluent.name == "ingdc" ||
           fluent.name == "-ingdc" ||
           fluent.name == "know" ||
           fluent.name == "-know" ||
           fluent.name == "inmeeting" ||
           fluent.name == "-inmeeting" ||
           fluent.name == "inmeetingornowhere" ||
           fluent.name == "-inmeetingornowhere" ||
           fluent.name == "allinmeeting" ||
           fluent.name == "-allinmeeting";
  }
};

struct NotPositionFluent {

  bool operator()(const bwi_kr_execution::AspFluent& fluent) const {

    return !(fluent.name == "at" ||
             fluent.name == "-at" ||
             fluent.name == "beside" ||
             fluent.name == "-beside" ||
             fluent.name == "facing" ||
             fluent.name == "-facing");
  }
};

bool goToInitialState(Client &client) {
  bwi_kr_execution::ExecutePlanGoal go_to_initial_state;

  bwi_kr_execution::AspRule initial_rule1;
  bwi_kr_execution::AspFluent front_of_lab_door;
  front_of_lab_door.name = "not facing";
  front_of_lab_door.variables.push_back("d3_414b1");
  initial_rule1.body.push_back(front_of_lab_door);
  
  bwi_kr_execution::AspRule initial_rule2;
  bwi_kr_execution::AspFluent outside_of_lab;
  outside_of_lab.name = "at";
  outside_of_lab.variables.push_back("l3_414b");
  initial_rule2.body.push_back(outside_of_lab);

  go_to_initial_state.aspGoal.push_back(initial_rule1);
  go_to_initial_state.aspGoal.push_back(initial_rule2);

  ROS_INFO("Going in front of the big lab door, I'll start from there!");

  client.sendGoalAndWait(go_to_initial_state);

  if (client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO_STREAM("I can't reach the initial position, giving up... ");
    return false;
  }
  
  ROS_INFO("Inital state achieved");

  return true;
}


void resetMemory() {

  ros::NodeHandle n;

  ROS_INFO("Resetting the knowledge base (forgetting everything)");

  ros::ServiceClient current_state_query = n.serviceClient<bwi_kr_execution::CurrentStateQuery>("current_state_query");
  bwi_kr_execution::CurrentStateQuery query;
  current_state_query.call(query);

  list<bwi_kr_execution::AspFluent> fluents_to_keep;
  remove_copy_if(query.response.answer.fluents.begin(), query.response.answer.fluents.end(), back_inserter(fluents_to_keep), NotPositionFluent());

  ros::ServiceClient forget_everything = n.serviceClient<std_srvs::Empty>("reset_state");
  std_srvs::Empty nothingness;
  forget_everything.call(nothingness);

  ros::ServiceClient put_fluents_back = n.serviceClient<bwi_kr_execution::UpdateFluents>("update_fluents");
  bwi_kr_execution::UpdateFluents toUpdate;

  toUpdate.request.fluents.insert(toUpdate.request.fluents.end(),fluents_to_keep.begin(), fluents_to_keep.end());

  put_fluents_back.call(toUpdate);

  ROS_INFO_STREAM("The oblivion feels good!");
}

int main(int argc, char**argv) {
  ros::init(argc, argv, "look_for_person");
  ros::NodeHandle n;

  ros::NodeHandle privateNode("~");
  string person;
  privateNode.param<string>("person",person,"peter");

  Client client("action_executor/execute_plan", true);
  client.waitForServer();

  while (ros::ok()) {

    if (!goToInitialState(client))
      return 0;


    resetMemory();


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

    goal.aspGoal.push_back(rule);


    ROS_INFO("sending goal");
    client.sendGoal(goal);
    client.waitForResult(ros::Duration(900.0)); //15 minutes to complete the task

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
  
  client.cancelAllGoals();

  return 0;
}
