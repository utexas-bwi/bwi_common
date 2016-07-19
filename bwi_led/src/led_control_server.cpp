/*******************************************************
*                    C Headers                         *
********************************************************/
#include <unistd.h>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>

/*******************************************************
*                    ROS Headers                       *
********************************************************/
#include <actionlib/server/simple_action_server.h>

/*******************************************************
*                   BWI_LED Headers                    *
********************************************************/
#include "ledcom.h"

/*******************************************************
*                   Service Headers                    *
********************************************************/
#include "bwi_led/led_clear.h"
#include "bwi_led/test_strip.h"
#include "bwi_led/set_first_five.h"
#include "bwi_led/set_every_fifth.h"

/*******************************************************
*                   Action Headers                     *
********************************************************/
#include "bwi_led/LEDAction.h"
#include "bwi_led/led_animations.h"

/*******************************************************
*                 Global Variables                     *
********************************************************/
LedCOM leds;
int led_count;

// Values hard coded for testing purposes
int left_segment_count = 20;
int left_front_start = 5;
int left_front_end = 24;

int right_segment_count = 20;
int right_front_start = 25;
int right_front_end = 44;

/*******************************************************
*                                                      *
*                   Action Server                      *
*                                                      *
********************************************************/
class LEDAction
{
protected:

  ros::NodeHandle nh_;

  actionlib::SimpleActionServer<bwi_led::LEDAction> as_; 
  std::string action_name_;

  bwi_led::LEDFeedback feedback_;
  bwi_led::LEDResult result_;

private:

  bool timeout = false;
  bool create_thread = true;
  bool success = true;

  void timeout_thread(int seconds)
  {
    timeout = false;

    ROS_INFO("Creating timeout thread with timeout: %d seconds.", seconds);
    boost::this_thread::sleep_for(boost::chrono::seconds{seconds});
    timeout = true;
    success = true;
    create_thread = true;
    ROS_INFO("Goal has reached timeout.");
  }

  void set_timeout(int time) 
  {
    if (time > 0 && create_thread) 
    {
      boost::thread timeout_t(boost::bind(&LEDAction::timeout_thread, this, time));
      create_thread = false;
    }
    else { return; }
  }

public:

  LEDAction(std::string name) :
    as_(nh_, name, boost::bind(&LEDAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~LEDAction(void)
  {
  }

  void executeCB(const bwi_led::LEDGoalConstPtr &goal)
  {
    ROS_INFO("%s: Executing LED Action: %d", action_name_.c_str(), goal->type);

    while(as_.isActive())
    {
      if (as_.isPreemptRequested())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        as_.setPreempted();
        leds.clear();
        ROS_INFO("Cleared LED Strip");
        success = false;
        timeout = false;
        create_thread = true;
        break;
      }

      if (!ros::ok())
      {
        ROS_INFO("%s: Preempted due to ROS failure", action_name_.c_str());
        as_.setPreempted();
        success = false;
        timeout = false;
        break;
      }

      switch(goal->type)
      {
        // Left Turn
        case 1: { 
                  while(!as_.isPreemptRequested() && !timeout)
                  {
                    LEDAction::set_timeout(goal->timeout);

                    for (int i = left_front_start; i < left_front_end;) 
                    {
                      if(timeout) { break; }

                      if (i == 0) 
                      {
                        leds.setHSV(i, 22, 1, .1);
                        leds.setHSV(i+1, 22, 1, .1);
                        leds.setHSV(i+2, 22, 1, .1);
                        leds.setHSV(i+3, 22, 1, .1);
                        leds.setHSV(i+4, 22, 1, .1);
                        i+=5;
                      }
                      else 
                      {
                        leds.setHSV(i, 22, 1, .1);
                        i+=1;
                      }
                      if (i > 0) 
                      {
                        leds.setRGB(i-5, 0, 0, 0);
                      }
                      leds.flush();
                      // Microseconds
                      usleep(100000);

                      if (i == left_front_end)
                      {
                        leds.setRGB(i, 0, 0, 0);
                        leds.setRGB(i-1, 0, 0, 0);
                        leds.setRGB(i-2, 0, 0, 0);
                        leds.setRGB(i-3, 0, 0, 0);
                        leds.setRGB(i-4, 0, 0, 0);
                        leds.flush();
                        // Microseconds
                        usleep(100000);
                      }
                    }
                  }
                  break;
                }

        // Right Turn
        case 2: {
                  while(!as_.isPreemptRequested() && !timeout)
                  {
                    LEDAction::set_timeout(goal->timeout);

                    for (int i = right_front_end; i > right_front_start;) 
                    {
                      if(timeout) { break; }

                      if (i == 0) 
                      {
                        leds.setHSV(i, 22, 1, .1);
                        leds.setHSV(i-1, 22, 1, .1);
                        leds.setHSV(i-2, 22, 1, .1);
                        leds.setHSV(i-3, 22, 1, .1);
                        leds.setHSV(i-4, 22, 1, .1);
                        i-=5;
                      }
                      else 
                      {
                        leds.setHSV(i, 22, 1, .1);
                        i-=1;
                      }
                      if (i > 0) 
                      {
                        leds.setRGB(i+5, 0, 0, 0);
                      }
                      leds.flush();
                      // Microseconds
                      usleep(100000);

                      if (i == right_front_start)
                      {
                        leds.setRGB(i, 0, 0, 0);
                        leds.setRGB(i+1, 0, 0, 0);
                        leds.setRGB(i+2, 0, 0, 0);
                        leds.setRGB(i+3, 0, 0, 0);
                        leds.setRGB(i+4, 0, 0, 0);
                        leds.flush();
                        // Microseconds
                        usleep(100000);
                      }
                    }
                  }
                  break;
                }
        // Reverse
        case 3:
                break;
        // Blocked
        case 4:
                break;
        // Up
        case 5:
                break;
        // Down
        case 6:
                break;

      }

      if(success || timeout)
      {
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        as_.setSucceeded();
        leds.clear();
        ROS_INFO("Cleared LED Strip");
        create_thread = true;
        timeout = false;
      }
    }
  }
};

/*******************************************************
*                                                      *
*                Service Callbacks                     *
*                                                      *
********************************************************/
bool clear_strip(bwi_led::led_clear::Request  &req,
                 bwi_led::led_clear::Response &res)
{
  leds.clear();
  ROS_INFO("Cleared LED Strip");
  return true;
}

bool test_strip(bwi_led::test_strip::Request  &req,
              bwi_led::test_strip::Response &res)
{
  leds.clear();
  sleep(1);

  for (int i=0; i < 360; i+=15) 
  {
    for (int l=0; l < led_count; l++) 
    {
      leds.setHSV(l, i, 1, .1);
    }
    leds.flush();
    sleep(1);
  }

  ROS_INFO("Tested HSV Colors on LED Strip");

  return true;
}

bool set_first_five(bwi_led::set_first_five::Request  &req,
                    bwi_led::set_first_five::Response &res)
{
  leds.clear();
  sleep(1);

  leds.setHSV(1, 240, 1, .1);
  leds.setHSV(2, 120, 1, .1);
  leds.setHSV(3, 240, 1, .1);
  leds.setHSV(4, 120, 1, .1);
  leds.setHSV(5, 240, 1, .1);

  leds.flush();
  sleep(1);

  ROS_INFO("Set first five LEDs on Strip");

  return true;
}

bool set_every_fifth(bwi_led::set_every_fifth::Request  &req,
                     bwi_led::set_every_fifth::Response &res)
{
  leds.clear();
  sleep(1);

  for (int l=5; l < led_count; l+=5) 
  {
    if (l%10 == 0)
    {
      leds.setHSV(l, 120, 1, .1);
    }
    else 
    {
      leds.setHSV(l, 250, 1, .1);
    }
  }
  leds.flush();
  sleep(1);

  ROS_INFO("Set every fifth LED on Strip");

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ledcom_server");
  ros::NodeHandle n;

  ros::NodeHandle privateNode("~");

  privateNode.param<int>("led_count",led_count,60);

  string serial_port;
  privateNode.param<string>("serial_port",serial_port,"/dev/metromini");

  leds.connect(serial_port, 115200);
  // Need to sleep at least 2 seconds to wait for connection to be established
  sleep(3);
  leds.setLEDCount(led_count);

  ros::ServiceServer clear_service = n.advertiseService("led_clear", clear_strip);
  ros::ServiceServer test_strip_service = n.advertiseService("test_strip", test_strip);
  ros::ServiceServer set_first_five_service = n.advertiseService("set_first_five", set_first_five);
  ros::ServiceServer set_every_fifth_service = n.advertiseService("set_every_fifth", set_every_fifth);

  LEDAction ledcom_server(ros::this_node::getName());

  ROS_INFO("Ready to control LED strip with %d leds.", led_count);
  ros::spin();

  return 0;
}