#include "plan_execution/ExecutePlanAction.h"
#include <actionlib/client/simple_action_client.h>
#include <boost/regex.hpp>
#include <boost/lexical_cast.hpp>
#include <ros/ros.h>
#include <string>

typedef actionlib::SimpleActionClient<plan_execution::ExecutePlanAction> Client;

std::string location;
std::string message;

int32_t speed;
std::string voice;
int32_t pitch;

void goToLocation() {  
  /*from go_to_location*/
  Client client("action_executor/execute_plan", true);
  client.waitForServer();
    
  plan_execution::ExecutePlanGoal goal;
    
  plan_execution::AspRule rule;
  plan_execution::AspFluent fluent;
  fluent.name = "not at";
    
  fluent.variables.push_back(location);
   
  rule.body.push_back(fluent);
  goal.aspGoal.push_back(rule);
    
  ROS_INFO("sending goal");
  client.sendGoal(goal);
    
  ros::Rate wait_rate(10);
  while(ros::ok() && !client.getState().isDone())
    wait_rate.sleep();
      
  if (!client.getState().isDone()) {
    ROS_INFO("Canceling goal");
    client.cancelGoal();
      //and wait for canceling confirmation
    for(int i = 0; !client.getState().isDone() && i < 50; ++i)
      wait_rate.sleep();
  }
  if (client.getState() == actionlib::SimpleClientGoalState::ABORTED) {
    ROS_INFO("Aborted");
  }
  else if (client.getState() == actionlib::SimpleClientGoalState::PREEMPTED) {
    ROS_INFO("Preempted");
  }
  else if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Succeeded!");
  }
  else
    ROS_INFO("Terminated");
}

void deliverMessage() { 
  std::string string_speed = boost::lexical_cast<std::string>(speed);
  std::string string_pitch = boost::lexical_cast<std::string>(pitch);

  boost::regex sanitizeVoice("[^a-z-]");
  std::string voiceReplacement = "";
  std::string clean_voice = boost::regex_replace(voice, sanitizeVoice, voiceReplacement);

  boost::regex sanitizeMessage("[^a-zA-Z\?!.,0-9-]");
  std::string messageReplacement = " ";
  std::string clean_message = boost::regex_replace(message, sanitizeMessage, messageReplacement);

  std::string command = "espeak -v " + clean_voice + " -s " 
      + string_speed + " -p " + string_pitch + " \"" + clean_message + "\"";
  
  system(command.c_str());
}

int main(int argc, char** argv) {
  //initialize the node
  ros::init(argc, argv, "message_delivery");

  ros::NodeHandle privateNode("~");

  privateNode.param<std::string>("location",location, /*default*/ ""); 
  privateNode.param<std::string>("message",message,"");
  privateNode.param<int32_t>("speed",speed,160);
  privateNode.param<std::string>("voice",voice,"default");
  privateNode.param<int32_t>("pitch",pitch,50);

  if (location.compare("") != 0) {
    goToLocation();
    deliverMessage();
  }
}
