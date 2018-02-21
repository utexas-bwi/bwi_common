#include "plan_execution/ExecutePlanAction.h"
#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
#include "bwi_virtour/GoToLocation.h"
#include "bwi_virtour/Authenticate.h"
#include "geometry_msgs/Twist.h"
#include "bwi_virtour/Rotate.h"

typedef actionlib::SimpleActionClient<plan_execution::ExecutePlanAction> Client;

Client* client;
ros::Publisher cmd_vel_pub;

using namespace std;

bool rotateRequest(bwi_virtour::Rotate::Request &req,
    bwi_virtour::Rotate::Response &res) {
  
  ROS_INFO("Received rotate request");
  float rotateDelta = req.rotateDelta;
  if (rotateDelta > 2) {
    rotateDelta = 2;
  } else if (rotateDelta < -2) {
    rotateDelta = -2;
  }

  ROS_INFO("authenticating user: %s", req.user.c_str());
  bwi_virtour::Authenticate::Request auth_req;
  auth_req.user = req.user;
  bwi_virtour::Authenticate::Response auth_res;

  if (ros::service::call("/tour_manager/authenticate", auth_req, auth_res)) {
    if (auth_res.result < 0) {
      res.result = -5;
      ROS_INFO("Authentication failed!");
      return true;
    }
  }
  
  geometry_msgs::Twist msg;
  msg.angular.z = rotateDelta;
  cmd_vel_pub.publish(msg);
  
  res.result = 1;
  return true;
}

int main(int argc, char**argv) {
  ros::init(argc, argv, "rotate_service_node");
  ros::NodeHandle n;

  cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  ros::ServiceServer service = n.advertiseService("rotate", rotateRequest);
  ROS_INFO("Rotate Service Started");

  ros::spin();
  ROS_INFO("Done spinning");
  return 0;
}
