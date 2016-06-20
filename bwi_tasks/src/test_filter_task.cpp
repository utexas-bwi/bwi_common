
#include "bwi_kr_execution/ExecutePlanAction.h"

#include <actionlib/client/simple_action_client.h>
#include <bwi_msgs/LogicalNavigationAction.h>
#include <bwi_msgs/DoorHandlerInterface.h>
#include <bwi_kr_execution/CurrentStateQuery.h>
#include <bwi_kr_execution/UpdateFluents.h>

#include <std_srvs/Empty.h>
#include <ros/ros.h>

#include <sstream>
#include <stdlib.h> //rand
#include <time.h> // time for rand

typedef actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction> Client;

using namespace std;

const size_t ep_per_phase = 15; 
const double max_time_per_episode = 300.0; //set to negative to remove the limit


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


bwi_kr_execution::ExecutePlanGoal initialStateFormula() {
  
  bwi_kr_execution::ExecutePlanGoal go_to_initial_state;
  
    bwi_kr_execution::AspRule initial_rule1;
  bwi_kr_execution::AspFluent front_of_lab_door;
  front_of_lab_door.name = "not facing";
  front_of_lab_door.variables.push_back("d3_414b1");
  initial_rule1.body.push_back(front_of_lab_door);
  
  bwi_kr_execution::AspRule initial_rule2;
  bwi_kr_execution::AspFluent outside_of_lab;
  outside_of_lab.name = "not at";
  outside_of_lab.variables.push_back("l3_414b");
  initial_rule2.body.push_back(outside_of_lab);

  go_to_initial_state.aspGoal.push_back(initial_rule1);
  go_to_initial_state.aspGoal.push_back(initial_rule2);
  
  return go_to_initial_state;
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

bwi_kr_execution::ExecutePlanGoal goToPlaceSpecific(const std::string& place, const std::string& where) {
  
  bwi_kr_execution::ExecutePlanGoal goal;

    //rule1
    bwi_kr_execution::AspRule rule1;
    bwi_kr_execution::AspFluent fluent1;
    fluent1.name = "not at";
    fluent1.variables.push_back(place);

    rule1.body.push_back(fluent1);
    goal.aspGoal.push_back(rule1);

    //rule2
    bwi_kr_execution::AspRule rule2;
    bwi_kr_execution::AspFluent fluent2;
    fluent2.name = "not facing";
    fluent2.variables.push_back(where);

    rule2.body.push_back(fluent2);
    goal.aspGoal.push_back(rule2);
    
    return goal;
}

void phaseTransition(int episode) {
  
  srand(time(NULL));

  ROS_INFO_STREAM("episode: " << episode);
  
 if(episode % ep_per_phase == 0) {  //every now and then change all doors
   
   //stringstream mkdir_cmd;
   //mkdir_cmd << "mkdir -p values" << (episode/ep_per_phase);
   
   //system(mkdir_cmd.str().c_str());
   
   //stringstream cp_cmd;
   //cp_cmd << "cp /var/tmp/my_bwi_action_execution/values/* values" << (episode/ep_per_phase);
   
   //system(cp_cmd.str().c_str());
  
   ros::NodeHandle n;
   ros::ServiceClient update_doors = n.serviceClient<bwi_msgs::DoorHandlerInterface>("update_doors");
   
   bwi_msgs::DoorHandlerInterface open_all_doors;
   open_all_doors.request.all_doors = true;
   open_all_doors.request.open = true;
   
   update_doors.call(open_all_doors);
   
/*  //at least one door of seminar room has to be open.. so either one is open, or the other, or both.
   bwi_msgs::DoorHandlerInterface close_516a;
   close_516a.request.door = "d3_516a";

   bwi_msgs::DoorHandlerInterface close_516b;
   close_516b.request.door = "d3_516b";

   int whichseminar = rand() % 3;
   if (whichseminar==0)
    update_doors.call(close_516a);
   else if (whichseminar==1)
    update_doors.call(close_516b);
  //else nothing to close  */


  //the four lab doors depend on the inside ones
   bwi_msgs::DoorHandlerInterface close_414b3;
   close_414b3.request.door = "d3_414b3";
  
   bwi_msgs::DoorHandlerInterface close_414a3;
   close_414a3.request.door = "d3_414a3";
   
   bwi_msgs::DoorHandlerInterface close_414b2;
   close_414b2.request.door = "d3_414b2";

   bwi_msgs::DoorHandlerInterface close_414a2;
   close_414a2.request.door = "d3_414a2";

   bwi_msgs::DoorHandlerInterface close_414b1;
   close_414b1.request.door = "d3_414b1";
   
   bwi_msgs::DoorHandlerInterface close_414a1;
   close_414a1.request.door = "d3_414a1";
  
   int howmanylab = rand() % 4 + 1; 
   
   if (howmanylab==1) {   //close 3 random outside doors
    int whichlab = rand() % 4;
    if (whichlab==0) {
      update_doors.call(close_414a2);
      update_doors.call(close_414a1);
      update_doors.call(close_414b1);
    }
    else if (whichlab==1) {
      update_doors.call(close_414b2);
      update_doors.call(close_414a1);
      update_doors.call(close_414b1);
    }
    else if (whichlab==2) {
      update_doors.call(close_414b2);
      update_doors.call(close_414a2);
      update_doors.call(close_414b1);      
    }
    else {
      update_doors.call(close_414b2);
      update_doors.call(close_414a1);
      update_doors.call(close_414a2);
    }
   }
   else if (howmanylab==2) {
    int sameside = rand() %2;
    if (sameside) {
      int whichside = rand() % 2;
      if (whichside==0) {
        update_doors.call(close_414a1);
        update_doors.call(close_414a2);
      }
      else {
        update_doors.call(close_414b2);
        update_doors.call(close_414b1);
      }
    }
    else { //different side
      int whichhere = rand() %2;
      if (whichhere == 0)
        update_doors.call(close_414a1);
      else
        update_doors.call(close_414a2);
      int whichthere = rand() %2;
      if (whichthere == 0)
        update_doors.call(close_414b1);
      else
        update_doors.call(close_414b2);
      int center = rand() % 2;
      if (center) {
        update_doors.call(close_414a3);
        update_doors.call(close_414b3);
      }
    }
   }
   else if (howmanylab==3) {
      int center = rand() % 2;
      if (center) {
        update_doors.call(close_414a3);
        update_doors.call(close_414b3);
      }
      int whichclose = rand() % 4;
      if (whichclose==0) 
        update_doors.call(close_414b1);
      else if (whichclose==1)
        update_doors.call(close_414b2);
      else if (whichclose==2) 
        update_doors.call(close_414a1);
      else 
        update_doors.call(close_414a2);
   }
   else {  //outside doors open
      int center = rand() % 2;
      if (center) {
        update_doors.call(close_414a3);
        update_doors.call(close_414b3);
      }
   }
    
 }

 if(episode % 3 == 1) {  //every round of tasks change seminar door

   ros::NodeHandle n;
   ros::ServiceClient update_doors = n.serviceClient<bwi_msgs::DoorHandlerInterface>("update_doors");
 
   //re open seminar doors, then maybe close some
   bwi_msgs::DoorHandlerInterface open_516a;
   open_516a.request.door = "d3_516a";
   open_516a.request.open = true;
   update_doors.call(open_516a);

   bwi_msgs::DoorHandlerInterface open_516b;
   open_516b.request.door = "d3_516b";
   open_516b.request.open = true;
   update_doors.call(open_516b);

  //at least one door of seminar room has to be open.. so either one is open, or the other, or both.
   bwi_msgs::DoorHandlerInterface close_516a;
   close_516a.request.door = "d3_516a";

   bwi_msgs::DoorHandlerInterface close_516b;
   close_516b.request.door = "d3_516b";

   int whichseminar = rand() % 3;
   if (whichseminar==0)
    update_doors.call(close_516a);
   else if (whichseminar==1)
    update_doors.call(close_516b);
  //else nothing to close
 }
  
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
  
  //Task chosen(in_front_of_lab, goToPlace("l3_414a"));

  Client client("action_executor/execute_plan", true);
  client.waitForServer();
  
  size_t task_counter = 0;

  while (ros::ok() && task_counter < ep_per_phase * 100) {
    
    phaseTransition(task_counter);
    
    //if (!goToInitialState(client, chosen.initial_state))
      //continue;
    
    //resetMemory();


    ROS_INFO("sending goal");
    //client.sendGoal(chosen.goal);




    if (task_counter % 3 == 0) 
      client.sendGoal(goToPlaceSpecific("l3_414a", "d3_414a1"));
    else if (task_counter % 3 == 1)
      client.sendGoal(goToPlaceSpecific("l3_414b", "d3_414b1"));
    else
      client.sendGoal(goToPlaceSpecific("l3_516", "d3_516a"));

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
