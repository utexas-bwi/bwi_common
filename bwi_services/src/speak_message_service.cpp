#include "bwi_services/SpeakMessage.h"
#include <ros/ros.h>
#include <string>
#include <boost/regex.hpp>
#include <boost/lexical_cast.hpp>

int32_t speed;
int32_t pitch;
std::string voice;

//text-to-speech service

bool speak_message(bwi_services::SpeakMessage::Request &req,
      bwi_services::SpeakMessage::Response &res) {
  
  std::string string_speed = boost::lexical_cast<std::string>(speed);
  std::string string_pitch = boost::lexical_cast<std::string>(pitch);

  boost::regex sanitizeVoice("[^a-z-]");
  std::string voiceReplacement = "";
  std::string clean_voice = boost::regex_replace(voice, sanitizeVoice, voiceReplacement);

  boost::regex sanitizeMessage("[^a-zA-Z\?!.,0-9-]");
  std::string messageReplacement = " ";
  std::string clean_message = boost::regex_replace(req.message, sanitizeMessage, messageReplacement);
 
  ROS_INFO("Saying:%s", clean_message.c_str());

  std::string command = "espeak -v " + clean_voice + " -s " 
    + string_speed + " -p " + string_pitch + " \"" + clean_message + "\"";
  
  int i = system(command.c_str());
 
  res.result = i;

  if (i != 0)
  {
    ROS_INFO("An error occured while calling espeak. Make sure espeak is installed and the voice is valid.");
    return false;
  }
  
  return true;
}

int main(int argc, char** argv) {
  //initialize the node
  ros::init(argc, argv, "speak_message_service_node");
  //private so that it can take params
  ros::NodeHandle nh("~");

  nh.param<int32_t>("speed", speed, 160);
  nh.param<int32_t>("pitch", pitch, 50);  
  nh.param<std::string>("voice", voice, "default");

  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("speak_message", speak_message);
  ROS_INFO("SpeakMessage Service started");
  ROS_INFO("Speed:%s",(boost::lexical_cast<std::string>(speed)).c_str());
  ROS_INFO("Pitch:%s",(boost::lexical_cast<std::string>(pitch)).c_str());
  ROS_INFO("Voice:%s",voice.c_str());

  ros::spin();
  
  return 0;
}
