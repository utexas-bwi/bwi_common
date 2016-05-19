
#include "bwi_kr_execution/ExecutePlanAction.h"

#include <actionlib/client/simple_action_client.h>
#include <bwi_msgs/LogicalNavigationAction.h>
#include <bwi_kr_execution/CurrentStateQuery.h>
#include <bwi_kr_execution/UpdateFluents.h>

#include <std_srvs/Empty.h>
#include <ros/ros.h>

#include <ctime> //to seed rand

typedef actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction> Client;

using namespace std;


struct Task {
  
  Task(const bwi_kr_execution::ExecutePlanGoal& initial_state, const bwi_kr_execution::ExecutePlanGoal& goal) :
        initial_state(initial_state), goal(goal) {}
  
  bwi_kr_execution::ExecutePlanGoal initial_state;
  bwi_kr_execution::ExecutePlanGoal goal;
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

bool goToInitialState(Client &client,const  bwi_kr_execution::ExecutePlanGoal& initial_state) {

  ROS_INFO("Going to the initial state");

  client.sendGoalAndWait(initial_state);

  if (client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO_STREAM("I can't reach the initial position, giving up... ");
    return false;
  }
  
  ROS_INFO("Inital state achieved");

  return true;
}

bwi_kr_execution::ExecutePlanGoal initialStateFormula() {
  
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
  
  return go_to_initial_state;
}
  
  
bwi_kr_execution::ExecutePlanGoal findPerson(const std::string& personName) {
  
  bwi_kr_execution::ExecutePlanGoal goal;

    bwi_kr_execution::AspRule main_rule;
    bwi_kr_execution::AspFluent fluent;
    fluent.name = "not ingdc";
    fluent.variables.push_back(personName);

    main_rule.body.push_back(fluent);


    bwi_kr_execution::AspFluent fluent_not;
    fluent_not.name = "not -ingdc";
    fluent_not.variables.push_back(personName);

    main_rule.body.push_back(fluent_not);
    
    bwi_kr_execution::AspRule flag_rule;
    bwi_kr_execution::AspFluent find_person_flag;
    find_person_flag.name = "findPersonTask";
    flag_rule.head.push_back(find_person_flag);

    goal.aspGoal.push_back(main_rule);
    goal.aspGoal.push_back(flag_rule);
    
    return goal;
}

bwi_kr_execution::ExecutePlanGoal goToPlace(const std::string& place) {
  
  bwi_kr_execution::ExecutePlanGoal goal;

    bwi_kr_execution::AspRule rule;
    bwi_kr_execution::AspFluent fluent;
    fluent.name = "not at";
    fluent.variables.push_back(place);

    rule.body.push_back(fluent);

    goal.aspGoal.push_back(rule);
    
    return goal;
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

  bwi_kr_execution::ExecutePlanGoal in_front_of_lab = initialStateFormula();
  
  vector<Task> tasks;
//  tasks.push_back(Task(in_front_of_lab, goToPlace("l2_302")));
  tasks.push_back(Task(in_front_of_lab, findPerson("matteo")));
//  tasks.push_back(Task(in_front_of_lab, findPerson("peter")));
//  tasks.push_back(Task(in_front_of_lab, goToPlace("l3_418")));
//  tasks.push_back(Task(in_front_of_lab, goToPlace("l3_414a")));
  //tasks.push_back(Task(in_front_of_lab, goToPlace("l3_418")));
  //tasks.push_back(Task(in_front_of_lab, goToPlace("l3_414b")));
  

  Client client("action_executor/execute_plan", true);
  client.waitForServer();
  
  srand(time(0));
  
  size_t task_counter = 0;

  while (ros::ok()) {
    
    Task &chosen = tasks[task_counter];
    task_counter = (task_counter + 1) % tasks.size();
    
    if (!goToInitialState(client, chosen.initial_state))
      continue;

   // resetMemory();


    ROS_INFO("sending goal");
    client.sendGoal(chosen.goal);
    
    ros::Rate rate(10);
    
    ros::Time start_time = ros::Time::now();
    bool too_late = false;
    
    while(ros::ok() && !client.getState().isDone()) {
      rate.sleep();
      ros::spinOnce(); //not sure this is necessary.
      
      if(!too_late && ((ros::Time::now() - start_time) > ros::Duration(900.0))) { //15 minutes to complete the task
        too_late = true;
        client.cancelGoal();
      }
    }

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
  
  ros::spinOnce();
  
  return 0;
}
