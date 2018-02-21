
#include "plan_execution/ExecutePlanAction.h"

#include <actionlib/client/simple_action_client.h>
#include <bwi_msgs/DoorHandlerInterface.h>
#include <plan_execution/CurrentStateQuery.h>
#include <plan_execution/UpdateFluents.h>

#include <std_srvs/Empty.h>
#include <ros/ros.h>

#include <sstream>

typedef actionlib::SimpleActionClient<plan_execution::ExecutePlanAction> Client;

using namespace std;

const size_t ep_per_phase = 200;
const double max_time_per_episode = 300.0; //set to negative to remove the limit


struct Task {
  
  Task(const plan_execution::ExecutePlanGoal& initial_state, const plan_execution::ExecutePlanGoal& goal) :
        initial_state(initial_state), goal(goal) {}
  
  plan_execution::ExecutePlanGoal initial_state;
  plan_execution::ExecutePlanGoal goal;
};

struct NotPositionFluent {

  bool operator()(const plan_execution::AspFluent& fluent) const {

    return !(fluent.name == "at" ||
             fluent.name == "-at" ||
             fluent.name == "beside" ||
             fluent.name == "-beside" ||
             fluent.name == "facing" ||
             fluent.name == "-facing");
  }
};

bool goToInitialState(Client &client,const  plan_execution::ExecutePlanGoal& initial_state) {

  ROS_INFO("Going to the initial state");

  client.sendGoalAndWait(initial_state);

  if (client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO_STREAM("I can't reach the initial position, giving up... ");
    return false;
  }
  
  ROS_INFO("Inital state achieved");

  return true;
}

plan_execution::ExecutePlanGoal initialStateFormula() {
  
  plan_execution::ExecutePlanGoal go_to_initial_state;
  
    plan_execution::AspRule initial_rule1;
  plan_execution::AspFluent front_of_lab_door;
  front_of_lab_door.name = "not facing";
  front_of_lab_door.variables.push_back("d3_414b1");
  initial_rule1.body.push_back(front_of_lab_door);
  
  plan_execution::AspRule initial_rule2;
  plan_execution::AspFluent outside_of_lab;
  outside_of_lab.name = "not at";
  outside_of_lab.variables.push_back("l3_414b");
  initial_rule2.body.push_back(outside_of_lab);

  go_to_initial_state.aspGoal.push_back(initial_rule1);
  go_to_initial_state.aspGoal.push_back(initial_rule2);
  
  return go_to_initial_state;
}

plan_execution::ExecutePlanGoal goToPlace(const std::string& place) {
  
  plan_execution::ExecutePlanGoal goal;

    plan_execution::AspRule rule;
    plan_execution::AspFluent fluent;
    fluent.name = "not at";
    fluent.variables.push_back(place);

    rule.body.push_back(fluent);

    goal.aspGoal.push_back(rule);
    
    return goal;
}

void phaseTransition(int episode) {
  
  ROS_INFO_STREAM("episode: " << episode);
  
 if(episode % ep_per_phase == 0) {
   
   stringstream mkdir_cmd;
   mkdir_cmd << "mkdir -p values" << (episode/ep_per_phase);
   
   system(mkdir_cmd.str().c_str());
   
   stringstream cp_cmd;
   cp_cmd << "cp /var/tmp/bwi_action_execution/values/* values" << (episode/ep_per_phase);
   
   system(cp_cmd.str().c_str());
  
   ros::NodeHandle n;
   ros::ServiceClient update_doors = n.serviceClient<bwi_msgs::DoorHandlerInterface>("update_doors");
   
   bwi_msgs::DoorHandlerInterface open_all_doors;
   open_all_doors.request.all_doors = true;
   open_all_doors.request.open = true;
   
   update_doors.call(open_all_doors);
   
   bwi_msgs::DoorHandlerInterface close_414b3;
  close_414b3.request.door = "d3_414b3";
  
   bwi_msgs::DoorHandlerInterface close_414a3;
   close_414a3.request.door = "d3_414a3";
   
   bwi_msgs::DoorHandlerInterface close_414a2;
   close_414a2.request.door = "d3_414a2";
   
   bwi_msgs::DoorHandlerInterface close_414a1;
   close_414a1.request.door = "d3_414a1";
   
   switch(episode) {
     case 0:       
              update_doors.call(close_414b3);
              update_doors.call(close_414a3);
              update_doors.call(close_414a2);
          break;
     case ep_per_phase:
              update_doors.call(close_414b3);
              update_doors.call(close_414a3);
              update_doors.call(close_414a1); 
          break;
     case ep_per_phase * 2:
              update_doors.call(close_414a1);
       break;
   }
   
 }
  
}


void resetMemory() {

  ros::NodeHandle n;

  ROS_INFO("Resetting the knowledge base (forgetting everything)");

  ros::ServiceClient current_state_query = n.serviceClient<plan_execution::CurrentStateQuery>("current_state_query");
  plan_execution::CurrentStateQuery query;
  current_state_query.call(query);

  list<plan_execution::AspFluent> fluents_to_keep;
  remove_copy_if(query.response.answer.fluents.begin(), query.response.answer.fluents.end(), back_inserter(fluents_to_keep), NotPositionFluent());

  ros::ServiceClient forget_everything = n.serviceClient<std_srvs::Empty>("reset_state");
  std_srvs::Empty nothingness;
  forget_everything.call(nothingness);

  ros::ServiceClient put_fluents_back = n.serviceClient<plan_execution::UpdateFluents>("update_fluents");
  plan_execution::UpdateFluents toUpdate;

  toUpdate.request.fluents.insert(toUpdate.request.fluents.end(),fluents_to_keep.begin(), fluents_to_keep.end());

  put_fluents_back.call(toUpdate);

  ROS_INFO_STREAM("The oblivion feels good!");
}

int main(int argc, char**argv) {
  ros::init(argc, argv, "look_for_person");
  ros::NodeHandle n;

  plan_execution::ExecutePlanGoal in_front_of_lab = initialStateFormula();
  
  Task chosen(in_front_of_lab, goToPlace("l3_414a"));
  

  Client client("action_executor/execute_plan", true);
  client.waitForServer();
  
  size_t task_counter = 0;

  while (ros::ok() && task_counter < ep_per_phase * 3) {
    
    phaseTransition(task_counter);
    
    if (!goToInitialState(client, chosen.initial_state))
      continue;
    
    resetMemory();


    ROS_INFO("sending goal");
    client.sendGoal(chosen.goal);
    
    ros::Rate rate(10);
    
    ros::Time start_time = ros::Time::now();
    bool too_late = false;
    
    while(ros::ok() && !client.getState().isDone()) {
      rate.sleep();
      ros::spinOnce(); //not sure this is necessary.
      
      if(max_time_per_episode >= 0) {
      
        if(!too_late && ((ros::Time::now() - start_time) > ros::Duration(max_time_per_episode))) {
          too_late = true;
          client.cancelGoal();
        }
      
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
    
    ++task_counter;
  }

  client.cancelAllGoals();
  
  ros::spinOnce();
  
  return 0;
}
