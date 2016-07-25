#include "bwi_services/SpeakMessage.h"
#include <ros/ros.h>
#include <string>
#include <boost/regex.hpp>
#include <boost/lexical_cast.hpp>
	
int speed;
std::string voice;
int pitch;

bool speak_message(bwi_services::SpeakMessage::Request &req,
		  bwi_services::SpeakMessage::Response &res) {
		
	boost::regex sanitizeNumber("[^0-9]");	
	std::string numberReplacement = "";
	std::string clean_speed = 
		boost::regex_replace(boost::lexical_cast<std::string>(speed), sanitizeNumber, numberReplacement);	
	std::string clean_pitch = 
		boost::regex_replace(boost::lexical_cast<std::string>(pitch), sanitizeNumber, numberReplacement);

	boost::regex sanitizeVoice("[^a-z-]");
	std::string voiceReplacement = "";
	std::string clean_voice = boost::regex_replace(voice, sanitizeVoice, voiceReplacement);
	
	boost::regex sanitizeMessage("[^a-zA-Z\?!.,0-9-]");
	std::string messageReplacement = " ";
	std::string clean_message = boost::regex_replace(req.message, sanitizeMessage, messageReplacement);


	std::string command = "espeak -v " + clean_voice + " -s " 
		+ clean_speed + " -p " + clean_pitch + " \"" + clean_message + "\"";
	system(command.c_str());

	return true;
}

int main(int argc, char** argv) {
	//initialize the node
	ros::init(argc, argv, "speak_message_service");
	ros::NodeHandle n;

	n.param<int>("speed",speed, /*default*/ 160);
        n.param<std::string>("voice",voice,"default");
	n.param<int>("pitch",pitch,50);	
	
	ros::ServiceServer service = n.advertiseService("speak_message", speak_message);
	ROS_INFO("SpeakMessage Service started");

	ros::spin();
	ROS_INFO("Done spinning");
	return 0;
}
