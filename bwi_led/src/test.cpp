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

using namespace std;

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_ledcom_server");

  ros::NodeHandle privateNode("~");

  int type;
  privateNode.param<int>("type",type,2);

  int timeout;
  privateNode.param<int>("timeout",timeout,10);

  actionlib::SimpleActionClient<bwi_led::LEDAction> ac("ledcom_server", true);

  ROS_INFO("Waiting for action server to start.");
  ac.waitForServer();

  ROS_INFO("Action server started, sending goal.");
  bwi_led::LEDGoal goal;


  switch(type){

    case 1: { 
              goal.type = bwi_led::led_animations::LEFT_TURN;
              break;
            }
    case 2: { 
              goal.type = bwi_led::led_animations::RIGHT_TURN;
              break;
            }
    case 3: { 
              goal.type = bwi_led::led_animations::REVERSE;
              break;
            }
    case 4: { 
              goal.type = bwi_led::led_animations::BLOCKED;
              break;
            }
    case 5: { 
              goal.type = bwi_led::led_animations::UP;
              break;
            }
    case 6: { 
              goal.type = bwi_led::led_animations::DOWN;
              break;
            }
  }

  goal.timeout = timeout;
  ac.sendGoal(goal);

  ROS_INFO("Action received goal: %s",ac.getState().toString().c_str());

  return 0;
}