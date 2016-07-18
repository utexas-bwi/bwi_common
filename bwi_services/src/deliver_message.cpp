#include "bwi_kr_execution/ExecutePlanAction.h"

#include <actionlib/client/simple_action_client.h>

#include "bwi_services/DeliverMessage.h"

#include <sound_play/sound_play.h>

#include <ros/ros.h>

#include <string>

typedef actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction> Client;

ros::NodeHandle *nh;

void goToLocation(std::string location) {  
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

void sleepok(int t) {
	if ((*nh).ok())
		sleep(t);
}

void deliverMessage(std::string message) {
	sound_play::SoundClient sc;
	sleepok(2);
	sc.say(message);
	sleepok(2);	
}

bool run_delivery(bwi_services::DeliverMessage::Request &req,
		  bwi_services::DeliverMessage::Request &res) {
	if ((req.location).compare("") != 0) {
		goToLocation(req.location);
		deliverMessage(req.message);
		return true;
	}
	
	return false;
}

int main(int argc, char** argv) {
	//initialize the node
	ros::init(argc, argv, "deliver_message");
	ros::NodeHandle n;
	nh = &n;
	
	ros::ServiceServer service = n.advertiseService("deliver_message", run_delivery);
	ros::spin();
	
	return 0;
}
