#include "ros/ros.h"
#include "bwi_services/DeliverMessage.h"
#include <cstdlib>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "deliver_message_client");
	if (argc != 2) {
		ROS_INFO("usage: deliver_message_client location message");
	}

 	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<bwi_services::DeliverMessage>("deliver_message");
	bwi_services::DeliverMessage srv;
	srv.request.location = argv[1];
	srv.request.message = argv[2];

	if (client.call(srv)) {
		ROS_INFO("Travelling to %s to deliver message: %s", argv[1], argv[2]);
  	}
  	else {
    		ROS_ERROR("Failed to call service");
    		return 1;
  	}

  	return 0;
}
