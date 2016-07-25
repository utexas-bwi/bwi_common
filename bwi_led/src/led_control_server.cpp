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

  // Thread which will sleep for specified time and set a timeout boolean
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
    // Creates a timeout thread only when time is valid and there is no other thread
    if (time > 0 && create_thread) 
    {
      boost::thread timeout_t(boost::bind(&LEDAction::timeout_thread, this, time));
      create_thread = false;
    }
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

    // Will run as long as goal is active
    while(as_.isActive())
    {
      // Preempted Execution Logic
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

      // ROS Failure Logic
      if (!ros::ok())
      {
        ROS_INFO("%s: Preempted due to ROS failure", action_name_.c_str());
        as_.setPreempted();
        success = false;
        timeout = false;
        break;
      }

      // Determines which animation to execute based on goal->type value
      switch(goal->type)
      {
        // Left Turn Animation
        case 1: { 
                  // Executes as long as timeout has not been reached, Goal is not Preempted, and ROS is OK 
                  while(!as_.isPreemptRequested() && !timeout && ros::ok())
                  {
                    LEDAction::set_timeout(goal->timeout);

                    // Creates a set of 3 leds which travels left along the strip

                    for (int i = left_front_start; i < left_front_end;) 
                    {
                      if(as_.isPreemptRequested() || timeout || !ros::ok()) { break; }

                      if (i == left_front_start) 
                      {
                        leds.setHSV(i, 22, 1, .1);
                        leds.setHSV(i+1, 22, 1, .1);
                        leds.setHSV(i+2, 22, 1, .1);
                        leds.setHSV(i+3, 22, 1, .1);
                        i+=4;
                      }
                      else 
                      {
                        leds.setHSV(i, 22, 1, .1);
                        i+=1;
                      }

                      if (i > left_front_start) 
                      {
                        leds.setRGB(i-4, 0, 0, 0);
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

                        i+=1;

                        leds.flush();
                        // Microseconds
                        usleep(100000);
                      }
                    }
                  }
                  break;
                }

        // Right Turn Animation
        case 2: {
                  // Executes as long as timeout has not been reached, Goal is not Preempted, and ROS is OK 
                  while(!as_.isPreemptRequested() && !timeout && ros::ok())
                  {
                    LEDAction::set_timeout(goal->timeout);

                    // Creates a set of 3 leds which travels right along the strip

                    for (int i = right_front_end; i > right_front_start;) 
                    {
                      if(as_.isPreemptRequested() || timeout || !ros::ok()) { break; }

                      if (i == right_front_end) 
                      {
                        leds.setHSV(i, 22, 1, .1);
                        leds.setHSV(i-1, 22, 1, .1);
                        leds.setHSV(i-2, 22, 1, .1);
                        leds.setHSV(i-3, 22, 1, .1);
                        i-=4;
                      }
                      else 
                      {
                        leds.setHSV(i, 22, 1, .1);
                        i-=1;
                      }

                      if (i < right_front_end) 
                      {
                        leds.setRGB(i+4, 0, 0, 0);
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

                        i-=1;

                        leds.flush();
                        // Microseconds
                        usleep(100000);
                      }
                    }
                  }
                  break;
                }

        // Reverse Animtion
        case 3: {
                  // Executes as long as timeout has not been reached, Goal is not Preempted, and ROS is OK 
                  while(!as_.isPreemptRequested() && !timeout && ros::ok())
                  {
                    LEDAction::set_timeout(goal->timeout);

                    // Creates a pulsing animation

                    // Increase brightness
                    for (float b = 0.0; b < 0.5; b += 0.02) 
                    {
                      if(as_.isPreemptRequested() || timeout || !ros::ok()) { break; }

                      for (int i = right_front_end; i > right_front_start; i--) 
                      {
                        leds.setHSV(i, 360, 1, b);
                      }

                      leds.flush();
                      // Microseconds
                      usleep(20000);
                    }
                    // Microseconds
                    usleep(500000);

                    // Decreases Brightness
                    for (float b = 0.5; b >= 0.0; b -= 0.02) 
                    {
                      if(timeout) { break; }

                      for (int i = right_front_end; i > right_front_start; i--) 
                      {
                        leds.setHSV(i, 360, 1, b);
                      }

                      leds.flush();
                      // Microseconds
                      usleep(20000);
                    }
                    // Microseconds
                    usleep(500000);
                  }
                  break;
                }

        // Blocked Animation
        case 4: {
                  bool first = true;

                  // Executes as long as timeout has not been reached, Goal is not Preempted, and ROS is OK 
                  while(!as_.isPreemptRequested() && !timeout && ros::ok())
                  {
                    LEDAction::set_timeout(goal->timeout);

                    // Creates a set of 2 leds which travels along a lighted strip

                    if(first) 
                    {
                      for (int i = right_front_end; i >= right_front_start; i--) 
                      {
                        leds.setHSV(i, 360, 1, .5);
                      }

                      leds.flush();
                      usleep(100000);

                      first = false;
                    }

                    for (int i = right_front_end; i >= right_front_start;) 
                    {
                      if(as_.isPreemptRequested() || timeout || !ros::ok()) { break; }

                      if (i == right_front_end) 
                      {
                        leds.setHSV(i, 360, 1, .1);
                        leds.setHSV(i-1, 360, 1, .1);
                        leds.setHSV(i-2, 360, 1, .1);
                        // leds.setHSV(i-3, 360, 1, .1);
                        // leds.setHSV(i-4, 360, 1, .1);
                        // i-=5;
                        i-=3;
                      }
                      else 
                      {
                        leds.setHSV(i, 360, 1, .1);
                        i-=1;
                      }

                      if (i < right_front_end) 
                      {
                        // leds.setHSV(i+5, 360, 1, .5);
                        leds.setHSV(i+3, 360, 1, .5);
                      }
                      leds.flush();
                      // Microseconds
                      usleep(100000);

                      if (i == right_front_start)
                      {
                        leds.setHSV(i, 360, 1, .5);
                        leds.setHSV(i+1, 360, 1, .5);
                        leds.setHSV(i+2, 360, 1, .5);
                        // leds.setHSV(i+3, 360, 1, .5);
                        // leds.setHSV(i+4, 360, 1, .5);

                        i-=1;

                        leds.flush();
                        // Microseconds
                        usleep(100000);
                      }
                    }
                  }
                  break;
                }
        // Up
        case 5:
                break;
        // Down
        case 6:
                break;

      }

      // Successful Execution Logic
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
  ROS_INFO("Testing Colors on LED Strip, will take about 30 seconds to complete.");

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

  leds.clear();
  ROS_INFO("Cleared LED Strip");
  ROS_INFO("Tested Colors on LED Strip");

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

  // Node Parameters
  privateNode.param<int>("led_count",led_count,60);

  string serial_port;
  privateNode.param<string>("serial_port",serial_port,"/dev/metromini");

  // Create parameters for segments

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