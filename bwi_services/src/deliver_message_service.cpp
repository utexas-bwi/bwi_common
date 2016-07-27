#include "bwi_services/DeliverMessage.h"
#include "bwi_services/GoToLocation.h"
#include "bwi_services/SpeakMessage.h"

#include <ros/ros.h>

#include <string>

ros::NodeHandle *n;

bool deliver_message(bwi_services::DeliverMessage::Request &req,
		  bwi_services::DeliverMessage::Response &res) {
	ros::ServiceClient goToLocationClient = (*n).serviceClient<bwi_services::GoToLocation>("go_to_location");	
	bwi_services::GoToLocation goToSrv;
  goToSrv.request.location = req.location;
	goToLocationClient.call(goToSrv);
	
  ROS_INFO("Location:%s", req.location.c_str());

  //called with header because node is private
	ros::ServiceClient speakMessageClient = (*n).serviceClient<bwi_services::SpeakMessage>("/speak_message_service/speak_message");  
	bwi_services::SpeakMessage speakSrv;
	speakSrv.request.message = req.message;

  ROS_INFO("Message:%s", req.message.c_str());

	speakMessageClient.call(speakSrv);	
	res.result = 1;
	
	return true;
}

int main(int argc, char** argv) {
	//initialize the node
	ros::init(argc, argv, "deliver_message_service");
	ros::NodeHandle nh;
	n = &nh;	

	ros::ServiceServer service = (*n).advertiseService("deliver_message", deliver_message);	
  ROS_INFO("DeliverMessage Service started");

	ros::spin();	
  ROS_INFO("Done spinning");
	return 0;
}

