#include "bwi_kr_execution/ExecutePlanAction.h"

#include <actionlib/client/simple_action_client.h>

#include <boost/regex.hpp>

#include <ros/ros.h>

#include <string>

typedef actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction> Client;

std::string location;
std::string message;

int speed;
std::string voice;
int pitch;

void goToLocation() {  
	/*from go_to_location*/
	Client client("action_executor/execute_plan", true);
	client.waitForServer();
	  
	bwi_kr_execution::ExecutePlanGoal goal;
	  
	bwi_kr_execution::AspRule rule;
	bwi_kr_execution::AspFluent fluent;
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

/*void sleepok(int t, ros::NodeHandle &nh)
{
	if (nh.ok())
		sleep(t);
}*/

void deliverMessage() {	
	
	boost::regex sanitizeNumber("[^0-9]");
        std::string numberReplacement = "";
        std::string clean_speed = boost::regex_replace(speed, sanitizeNumber, numberReplacement);
        std::string clean_pitch = boost::regex_replace(pitch, sanitizeNumber, numberReplacement);

        boost::regex sanitizeVoice("[^a-z-]");
        std::string voiceReplacement = "";
        std::string clean_message = boost::regex_replace(voice, sanitizeVoice, voiceReplacement);
	
	boost::regex sanitizeMessage("[^a-zA-Z\?!0-9-]");
        std::string messageReplacement = " ";
        std::string clean_message = boost::regex_replace(req.message, sanitizeMessage, messageReplacement);


        std::string command = "espeak -v " + voice + " -s " + speed + " -p " + pitch + " \"" + clean_message + "\"";
        system(command.c_str());
}

int main(int argc, char** argv) {
	//initialize the node
	ros::init(argc, argv, "message_delivery");

	ros::NodeHandle privateNode("~");

	privateNode.param<std::string>("location",location, /*default*/ ""); 
	privateNode.param<std::string>("message",message,"");
	privateNode.param<int64>("speed",speed, "160");
        privateNode.param<std::string>("voice",voice,"default");
        privateNode.param<int64>("pitch",pitch,"50");

	if (location.compare("") != 0) {
		goToLocation();
		deliverMessage();
	}
}
