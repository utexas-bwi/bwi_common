/*******************************************************
*                    ROS Headers                       *
********************************************************/
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

/*******************************************************
*                   Action Headers                     *
********************************************************/
#include <bwi_led/LEDAction.h>
#include "bwi_led/led_animations.h"

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_ledcom_server");

  actionlib::SimpleActionClient<bwi_led::LEDAction> ac("ledcom_server", true);

  ROS_INFO("Waiting for action server to start.");
  ac.waitForServer();

  ROS_INFO("Action server started, sending goal.");
  bwi_led::LEDGoal goal;
  goal.type = bwi_led::led_animations::RIGHT_TURN;
  goal.timeout = 10;
  ac.sendGoal(goal);

  ROS_INFO("Action received goal: %s",ac.getState().toString().c_str());

  return 0;
}