#include <actionlib_tutorials/FibonacciAction.h>
#include <bwi_interruptable_action_server/interruptable_action_server.h>
#include <ros/ros.h>

using namespace bwi_interruptable_action_server;

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "fibonacci_interruptable_action_server");
  ros::NodeHandle nh;

  InterruptableActionServer<actionlib_tutorials::FibonacciAction> as(nh, "fibonacci");
  as.spin();
  
  return 0;
}
