
#include "bwi_kr_execution/ExecutePlanAction.h"
#include <bwi_kr_execution/UpdateFluents.h>

#include <actionlib/client/simple_action_client.h>

#include <ros/ros.h>
#include <time.h> 

typedef actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction> Client;

using namespace std;

bwi_kr_execution::ExecutePlanGoal navigationGoal(const string location) {
  bwi_kr_execution::ExecutePlanGoal goal;

    bwi_kr_execution::AspRule rule;
    bwi_kr_execution::AspFluent fluent;
    fluent.name = "not facing";

    fluent.variables.push_back(location);

    rule.body.push_back(fluent);
    goal.aspGoal.push_back(rule);

  return goal;
}

bwi_kr_execution::ExecutePlanGoal findPersonGoal(const string person) {
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


    bwi_kr_execution::AspRule rule_nav;
    bwi_kr_execution::AspFluent fluent_nav;
    fluent_nav.name = "not facing";

    fluent_nav.variables.push_back("d3_414b2");

    rule_nav.body.push_back(fluent_nav);

    goal.aspGoal.push_back(rule);
    //goal.aspGoal.push_back(rule_nav);

  return goal;
}

int main(int argc, char**argv) {
  ros::init(argc, argv, "visit_and_ask_node");
  ros::NodeHandle n;

  ros::NodeHandle privateNode("~");

  srand (time(NULL));
  
  
  /*string locationA;
  privateNode.param<string>("a",locationA,"d3_414b1");

  string locationB
  privateNode.param<string>("b",locationB,"d3_414b2");*/
  
  std::vector<string> doors;
  doors.push_back("d3_414b1");
  doors.push_back("d3_414b2");
  doors.push_back("d3_414a1");
  doors.push_back("d3_414a2");

  std::vector<string> people;
  people.push_back("peter");
  people.push_back("jivko");
  people.push_back("garrett");
  people.push_back("justin");
  people.push_back("yuqian");

  int current_door = 0;
  int current_person = 0;

  Client client("/action_executor/execute_plan", true);
  client.waitForServer();

  ros::ServiceClient krClient = n.serviceClient<bwi_kr_execution::UpdateFluents> ( "update_fluents" );
    krClient.waitForExistence();

  bool fromAtoB = true;

  while (ros::ok()) {
    bwi_kr_execution::ExecutePlanGoal goal;

    float r = ((float) rand() / (RAND_MAX));
    r=0;
    if (r >= 0.7) {
      string location = doors.at(current_door);
      current_door++;
      if (current_door >= (int)doors.size())
          current_door = 0;

      ROS_INFO_STREAM("going to " << location);

      goal = navigationGoal(location);
    }
    else {
      string person = people.at(current_person);
      current_person++;
      if (current_person >= (int)people.size())
          current_person = 0;

      bwi_kr_execution::UpdateFluents uf;
      bwi_kr_execution::AspFluent find_person_flag;
      find_person_flag.name = "findPersonTask";
      find_person_flag.variables.push_back(person);
      uf.request.fluents.push_back(find_person_flag);
      krClient.call(uf);

      ROS_INFO_STREAM("looking for " << person);

      goal = findPersonGoal(person);
    }
    

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
