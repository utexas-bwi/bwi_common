#include "bwi_services/SpeakMessage.h"
#include <ros/ros.h>
#include <string>
#include <boost/regex.hpp>
	
int speed;
std::string voice;
int pitch;

bool speak_message(bwi_services::SpeakMessage::Request &req,
		  bwi_services::SpeakMessage::Request &res) {

	boost::regex sanitizeMessage("[^a-zA-Z\?!0-9-]");
	std::string messageReplacement = " ";
	std::string clean_message = boost::regex_replace(req.message, sanitizeMessage, messageReplacement);


	std::string command = "espeak -v " + voice + " -s " + speed + " -p " + pitch + " \"" + clean_message + "\"";
	system(command.c_str());

	return true;
}

int main(int argc, char** argv) {
	//initialize the node
	ros::init(argc, argv, "speak_message_service");
	ros::NodeHandle n;

        n.param<int64>("speed",speed, /*default*/ "160");
        n.param<std::string>("voice",voice,"default");
	n.param<int64>("pitch",pitch,"50");
	
	boost::regex sanitizeNumber("[^0-9]");	
	std::string numberReplacement = "";
	std::string clean_speed = boost::regex_replace(speed, sanitizeNumber, numberReplacement);	
	std::string clean_pitch = boost::regex_replace(pitch, sanitizeNumber, numberReplacement);

	boost::regex sanitizeVoice("[^a-z-]");
	std::string voiceReplacement = "";
	std::string clean_message = boost::regex_replace(voice, sanitizeVoice, voiceReplacement);
	
	ros::ServiceServer service = n.advertiseService("speak_message", speak_message);
	ROS_INFO("SpeakMessage Service started");

	ros::spin();
	ROS_INFO("Done spinning");
	return 0;
}
