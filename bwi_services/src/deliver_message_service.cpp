#include "bwi_services/DeliverMessage.h"
#include "bwi_services/GoToLocation.h"
#include "bwi_services/SpeakMessage.h"
#include <ros/ros.h>
#include <string>

ros::NodeHandle *n;

//travels to location and delivers message

bool deliver_message(bwi_services::DeliverMessage::Request &req,
      bwi_services::DeliverMessage::Response &res) {
  
  ros::ServiceClient goToLocationClient = (*n).serviceClient<bwi_services::GoToLocation>("/bwi_services/go_to_location"); 
  bwi_services::GoToLocation goToSrv;
  goToSrv.request.location = req.location;
  goToLocationClient.call(goToSrv);
  ROS_INFO("Location:%s", req.location.c_str());
  
  //called with header because node is private
  ros::ServiceClient speakMessageClient = (*n).serviceClient<bwi_services::SpeakMessage>("/speak_message_service/speak_message");  
  bwi_services::SpeakMessage speakSrv;
  speakSrv.request.message = req.message;
  speakMessageClient.call(speakSrv);  
  ROS_INFO("Message:%s", req.message.c_str());
  
  res.go_to_result = goToSrv.response.result;
  res.speak_result = speakSrv.response.result;
  
  return res.go_to_result == 1 && res.speak_result == 0;
}

int main(int argc, char** argv) {
  //initialize the node
  ros::init(argc, argv, "deliver_message_service_node");
  ros::NodeHandle nh;
  n = &nh;  

  ros::ServiceServer service = (*n).advertiseService("/bwi_services/deliver_message", deliver_message); 
  ROS_INFO("DeliverMessage Service started");

  ros::spin();  
  ROS_INFO("Done spinning");
  return 0;
}

