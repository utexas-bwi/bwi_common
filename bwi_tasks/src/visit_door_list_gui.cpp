/*
 * Example task intended to introduce new students
 * to the GUI and basic logical navigation
 * 
 */ 


#include "plan_execution/ExecutePlanAction.h"

#include <actionlib/client/simple_action_client.h>

#include <ros/ros.h>

#include <bwi_msgs/QuestionDialog.h>

typedef actionlib::SimpleActionClient<plan_execution::ExecutePlanAction> Client;

using namespace std;

int main(int argc, char**argv) {
  ros::init(argc, argv, "visit_door_list_gui");
  ros::NodeHandle n;

  ros::NodeHandle privateNode("~");
  
  std::vector<string> doors;

  doors.push_back("d3_414b1");
  doors.push_back("d3_414b2");
  doors.push_back("d3_414a1");
  doors.push_back("d3_414a2");
  doors.push_back("d3_418");
  int current_door = 0;

  Client client("/action_executor/execute_plan", true);
  client.waitForServer();

  ros::ServiceClient client_gui = n.serviceClient<bwi_msgs::QuestionDialog>("/question_dialog");
  
  while (ros::ok()) {
	  
	//use gui to decide which door to go to
	bwi_msgs::QuestionDialog question;
	question.request.type = question.request.CHOICE_QUESTION;
	
	question.request.message = "Please select the next door:";
	for (unsigned int k = 0; k < doors.size(); k++){
		question.request.options.push_back(doors.at(k));
	}
	question.request.timeout = 30.0;
	
	if (client_gui.call(question))
    {
		if (question.response.index >= 0){
			current_door = question.response.index;
		}
		else {
			ROS_INFO("No response detected, defaulting to 0");
			current_door = 0;
		}
	}
	else
	{
		ROS_ERROR("Failed to call service /question_dialog");
		return 1;
	}

    string location = doors.at(current_door);

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
