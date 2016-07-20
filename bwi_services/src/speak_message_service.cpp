#include "bwi_services/SpeakMessage.h"
#include <ros/ros.h>
#include <string>
#include <boost/regex.hpp>

bool speak_message(bwi_services::SpeakMessage::Request &req,
		  bwi_services::SpeakMessage::Request &res) {
	boost::regex sanitize("[^a-zA-Z\?!0-9-.,]");
	std::string replacement = " ";
	std::string clean_message = boost::regex_replace(req.message, sanitize, replacement);
	std::string command = "espeak \"" + clean_message + "\"";
	system(command.c_str());

	return true;
}

int main(int argc, char** argv) {
	//initialize the node
	ros::init(argc, argv, "speak_message_service");
	ros::NodeHandle n;
	
	ros::ServiceServer service = n.advertiseService("speak_message", speak_message);
	ROS_INFO("SpeakMessage Service started");

	ros::spin();
	ROS_INFO("Done spinning");
	return 0;
}
