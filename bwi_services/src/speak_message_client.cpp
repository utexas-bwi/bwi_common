#include "ros/ros.h"
#include "bwi_services/SpeakMessage.h"
#include <cstdlib>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "deliver_message_client");

 	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<bwi_services::SpeakMessage>("/speak_message_service/speak_message");
	bwi_services::SpeakMessage srv;
	srv.request.message = argv[1];

	if (client.call(srv)) {
		ROS_INFO("Delivering message %s", argv[1]);
  	}
  	else {
    		ROS_ERROR("Failed to call service");
    		return 1;
  	}

  	return 0;
}
