/*******************************************************
*                    C Headers                         *
********************************************************/
#include <unistd.h>
#include <signal.h>

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
#include "bwi_led/set_camera_on.h"
#include "bwi_led/set_camera_off.h"

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
string serial_port;
bool camera_on = false;

// LED Segments
int front_center_left_start;
int front_center_left_end;

int front_center_right_start;
int front_center_right_end;

int back_center_left_start;
int back_center_left_end;

int back_center_right_start;
int back_center_right_end;

int front_left_beam_start;
int front_left_beam_end;

int front_right_beam_start;
int front_right_beam_end ;

int back_left_beam_start;
int back_left_beam_end;

int back_right_beam_start;
int back_right_beam_end;

int camera_indicator_start;
int camera_indicator_end;

int back_center_start;
int back_center_end;


/*******************************************************
*                                                      *
*                 Helper Functions                     *
*                                                      *
********************************************************/
void check_camera_status()
{
  if(camera_on)
  {
    leds.clear();
    // Microseconds
    usleep(100000);

    for (int i = camera_indicator_start; i <= camera_indicator_end; i++)
    {
      leds.setHSV(i, 240, 1, .1);
    }

    leds.flush();
    // Microseconds
    usleep(100000);

    ROS_INFO("Set camera indicator on strip");
  }
}


void service_reconnect()
{
  ROS_ERROR("Service execution failed, unable to write to microcontroller,");
  ROS_ERROR("Ensure LED microcontroller is connected.");
  ROS_ERROR("Attempting to reconnect to LED microcontroller.");

  leds.connect(serial_port, 115200);
  // Need to sleep at least 2 seconds to wait for connection to be established
  sleep(3);
  leds.setLEDCount(led_count);

  ROS_INFO("Reconnected to led strip, ready to control LED strip with %d leds.", led_count);

  leds.clear();
  ROS_INFO("Cleared LED Strip");
  check_camera_status();
}

void ledSigintHandler(int sig)
{
  camera_on = false;
  ros::shutdown();
}

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

public:

  ros::Timer timeout_timer;

  bool timeout = false;
  bool success = true;

  void timerCallback(const ros::TimerEvent& event)
  {
    timeout = true;
    success = true;
    ROS_INFO("Goal has reached timeout.");
  }

  LEDAction(std::string name) :
    as_(nh_, name, boost::bind(&LEDAction::executeCB, this, _1), false),
    action_name_(name)
  {
    timeout_timer = nh_.createTimer(ros::Duration(100), boost::bind(&LEDAction::timerCallback, this, _1));
    timeout_timer.stop();
    as_.start();
  }

  ~LEDAction(void)
  {
  }

  void executeCB(const bwi_led::LEDGoalConstPtr &goal)
  {
    ROS_INFO("%s: Executing LED Action: %d", action_name_.c_str(), goal->type);

    leds.clear();
    // Microseconds
    usleep(100000);

    timeout_timer.stop();

    if (goal->timeout > 0) 
    {
      timeout = false;
      ROS_INFO("Creating timeout for: %d seconds.", goal->timeout);
      timeout_timer.setPeriod(ros::Duration(goal->timeout));
      timeout_timer.start();
    }

    try
    {
      // Will run as long as goal is active
      while(as_.isActive())
      {
        // Preempted Execution Logic
        if (as_.isPreemptRequested())
        {
          ROS_INFO("%s: Preempted", action_name_.c_str());
          as_.setPreempted();
          timeout_timer.stop();
          leds.clear();
          ROS_INFO("Cleared LED Strip");
          success = false;
          timeout = false;
          check_camera_status();
          break;
        }

        // ROS Failure Logic
        if (!ros::ok())
        {
          ROS_INFO("%s: Preempted due to ROS failure", action_name_.c_str());
          as_.setPreempted();
          timeout_timer.stop();
          leds.clear();
          ROS_INFO("Cleared LED Strip");
          check_camera_status();
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
                      // Creates an animation of leds which travels left along the strip

                      int j = back_center_left_start+1;

                      for (int i = front_center_left_start-1; i < front_center_left_end+1;) 
                      {
                        // Terminate goal if preempted, timeout is reached, or ros fails
                        if(as_.isPreemptRequested() || timeout || !ros::ok()) { break; }

                        if (i == front_center_left_start-1) 
                        {
                          leds.setHSV(i, 22, 1, .1);
                          leds.setHSV(i+1, 22, 1, .1);

                          leds.setHSV(j, 22, 1, .1);
                          leds.setHSV(j-1, 22, 1, .1);

                          i+=2;
                          j-=2;
                        }
                        else 
                        {
                          leds.setHSV(i, 22, 1, .1);
                          leds.setHSV(j, 22, 1, .1);
                          i+=1;
                          j-=1;
                        }

                        if (i > front_center_left_start-1) 
                        {
                          leds.setRGB(i-2, 0, 0, 0);
                          leds.setRGB(j+2, 0, 0, 0);
                        }

                        leds.flush();
                        // Microseconds
                        usleep(100000);

                        if (i == front_center_left_end+1)
                        {
                          leds.setRGB(i, 0, 0, 0);
                          leds.setRGB(i-1, 0, 0, 0);

                          leds.setRGB(j, 0, 0, 0);
                          leds.setRGB(j+1, 0, 0, 0);

                          i+=1;
                          j-=1;

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
                      // Creates an animation of leds which travels right along the strip

                      int j = front_center_right_start+1;

                      for (int i = back_center_right_start-1; i < back_center_right_end+1;) 
                      {
                        // Terminate goal if preempted, timeout is reached, or ros fails
                        if(as_.isPreemptRequested() || timeout || !ros::ok()) { break; }

                        if (i == back_center_right_start-1) 
                        {
                          leds.setHSV(i, 22, 1, .1);
                          leds.setHSV(i+1, 22, 1, .1);

                          leds.setHSV(j, 22, 1, .1);
                          leds.setHSV(j-1, 22, 1, .1);

                          i+=2;
                          j-=2;
                        }
                        else 
                        {
                          leds.setHSV(i, 22, 1, .1);
                          leds.setHSV(j, 22, 1, .1);
                          i+=1;
                          j-=1;
                        }

                        if (i > back_center_right_start-1) 
                        {
                          leds.setRGB(i-2, 0, 0, 0);
                          leds.setRGB(j+2, 0, 0, 0);
                        }

                        leds.flush();
                        // Microseconds
                        usleep(100000);

                        if (i == back_center_right_end+1)
                        {
                          leds.setRGB(i, 0, 0, 0);
                          leds.setRGB(i-1, 0, 0, 0);

                          leds.setRGB(j, 0, 0, 0);
                          leds.setRGB(j+1, 0, 0, 0);

                          i+=1;
                          j-=1;

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
                      // Creates a pulsing animation

                      // Increase brightness
                      for (float b = 0.0; b < 0.5; b += 0.02) 
                      {
                        // Terminate goal if preempted, timeout is reached, or ros fails
                        if(as_.isPreemptRequested() || timeout || !ros::ok()) { break; }

                        int j = back_left_beam_start;
                        int k = back_center_end;

                        for (int i = back_right_beam_end; i >= back_right_beam_start; i--) 
                        {
                          leds.setHSV(i, 360, 1, b);
                          leds.setHSV(j, 360, 1, b);

                          if (k >= back_center_start)
                          {
                            leds.setHSV(k, 360, 1, b);
                            k--;
                          }

                          j--;
                        }

                        leds.flush();
                        // Microseconds
                        usleep(22500);
                      }
                      // Microseconds
                      usleep(500000);

                      // Decreases Brightness
                      for (float b = 0.5; b >= 0.0; b -= 0.02) 
                      {
                        // Terminate goal if preempted, timeout is reached, or ros fails
                        if(as_.isPreemptRequested() || timeout || !ros::ok()) { break; }

                        int j = back_left_beam_start;
                        int k = back_center_end;

                        for (int i = back_right_beam_end; i >= back_right_beam_start; i--) 
                        {
                          leds.setHSV(i, 360, 1, b);
                          leds.setHSV(j, 360, 1, b);

                          if (k >= back_center_start)
                          {
                            leds.setHSV(k, 360, 1, b);
                            k--;
                          }

                          j--;
                        }

                        leds.flush();
                        // Microseconds
                        usleep(22500);
                      }
                      // Microseconds
                      usleep(500000);
                    }
                    break;
                  }

          // Blocked Animation
          case 4: {
                    // Turns on strip 
                    for (int i = led_count; i >= 0; i--) 
                    {
                      leds.setHSV(i, 360, 1, .5);
                    }
                    leds.flush();
                    // Microseconds
                    usleep(100000);

                    // Executes as long as timeout has not been reached, Goal is not Preempted, and ROS is OK 
                    while(!as_.isPreemptRequested() && !timeout && ros::ok())
                    {
                      // Creates an animation of leds which travels along a lighted strip

                      for (int i = led_count; i >= 0;) 
                      {
                        // Terminate goal if preempted, timeout is reached, or ros fails
                        if(as_.isPreemptRequested() || timeout || !ros::ok()) { break; }

                        if (i == led_count) 
                        {
                          leds.setHSV(i, 360, 1, .1);
                          leds.setHSV(i-1, 360, 1, .1);
                          leds.setHSV(i-2, 360, 1, .1);
                          leds.setHSV(i-3, 360, 1, .1);
                          leds.setHSV(i-4, 360, 1, .1);
                          i-=5;
                        }
                        else 
                        {
                          leds.setHSV(i, 360, 1, .1);
                          i-=1;
                        }

                        if (i < led_count) 
                        {
                          leds.setHSV(i+5, 360, 1, .5);
                        }
                        leds.flush();
                        // Microseconds
                        usleep(100000);

                        if (i == 0)
                        {
                          leds.setHSV(i, 360, 1, .5);
                          leds.setHSV(i+1, 360, 1, .5);
                          leds.setHSV(i+2, 360, 1, .5);
                          leds.setHSV(i+3, 360, 1, .5);
                          leds.setHSV(i+4, 360, 1, .5);

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
          case 5: {
                    // Executes as long as timeout has not been reached, Goal is not Preempted, and ROS is OK 
                    while(!as_.isPreemptRequested() && !timeout && ros::ok())
                    {
                      // Creates an animation of leds which travels up along the strip

                      int l = front_left_beam_start;
                      int k = front_right_beam_start;
                      int j = back_left_beam_start;

                      for (int i = back_right_beam_start; i <= back_right_beam_end;)
                      {
                        // Terminate goal if preempted, timeout is reached, or ros fails
                        if(as_.isPreemptRequested() || timeout || !ros::ok()) { break; }

                        if (i == back_right_beam_start)
                        {
                          leds.setHSV(i, 240, 1, .1);
                          leds.setHSV(i+1, 240, 1, .1);
                          leds.setHSV(i+2, 240, 1, .1);
                          leds.setHSV(i+3, 240, 1, .1);

                          leds.setHSV(l, 240, 1, .1);
                          leds.setHSV(l+1, 240, 1, .1);
                          leds.setHSV(l+2, 240, 1, .1);
                          leds.setHSV(l+3, 240, 1, .1);

                          leds.setHSV(k, 240, 1, .1);
                          leds.setHSV(k-1, 240, 1, .1);
                          leds.setHSV(k-2, 240, 1, .1);
                          leds.setHSV(k-3, 240, 1, .1);

                          leds.setHSV(j, 240, 1, .1);
                          leds.setHSV(j-1, 240, 1, .1);
                          leds.setHSV(j-2, 240, 1, .1);
                          leds.setHSV(j-3, 240, 1, .1);

                          i+=4;
                          l+=4;
                          k-=4;
                          j-=4;
                        }
                        else
                        {
                          leds.setHSV(i, 240, 1, .1);
                          leds.setHSV(l, 240, 1, .1);
                          leds.setHSV(k, 240, 1, .1);
                          leds.setHSV(j, 240, 1, .1);
                          i+=1;
                          l+=1;
                          k-=1;
                          j-=1;
                        }

                        if (i > back_right_beam_start)
                        {
                          leds.setRGB(i-4, 0, 0, 0);
                          leds.setRGB(l-4, 0, 0, 0);
                          leds.setRGB(k+4, 0, 0, 0);
                          leds.setRGB(j+4, 0, 0, 0);
                        }

                        leds.flush();
                        // Microseconds
                        usleep(100000);

                        if (i == back_right_beam_end)
                        {
                          leds.setRGB(i, 0, 0, 0);
                          leds.setRGB(i-1, 0, 0, 0);
                          leds.setRGB(i-2, 0, 0, 0);
                          leds.setRGB(i-3, 0, 0, 0);

                          leds.setRGB(l, 0, 0, 0);
                          leds.setRGB(l-1, 0, 0, 0);
                          leds.setRGB(l-2, 0, 0, 0);
                          leds.setRGB(l-3, 0, 0, 0);

                          leds.setRGB(k, 0, 0, 0);
                          leds.setRGB(k+1, 0, 0, 0);
                          leds.setRGB(k+2, 0, 0, 0);
                          leds.setRGB(k+3, 0, 0, 0);

                          leds.setRGB(j, 0, 0, 0);
                          leds.setRGB(j+1, 0, 0, 0);
                          leds.setRGB(j+2, 0, 0, 0);
                          leds.setRGB(j+3, 0, 0, 0);

                          i+=1;
                          l+=1;
                          k-=1;
                          j-=1;

                          leds.flush();
                          // Microseconds
                          usleep(100000);
                        }
                      }
                    }
                    break;
                  }
          // Down
          case 6: {
                    // Executes as long as timeout has not been reached, Goal is not Preempted, and ROS is OK 
                    while(!as_.isPreemptRequested() && !timeout && ros::ok())
                    {
                      // Creates an animation of leds which travels down along the strip

                      int l = front_left_beam_end;
                      int k = front_right_beam_end;
                      int j = back_left_beam_end;

                      for (int i = back_right_beam_end; i >= back_right_beam_start;)
                      {
                        // Terminate goal if preempted, timeout is reached, or ros fails
                        if(as_.isPreemptRequested() || timeout || !ros::ok()) { break; }

                        if (i == back_right_beam_end)
                        {
                          leds.setHSV(i, 240, 1, .1);
                          leds.setHSV(i-1, 240, 1, .1);
                          leds.setHSV(i-2, 240, 1, .1);
                          leds.setHSV(i-3, 240, 1, .1);

                          leds.setHSV(l, 240, 1, .1);
                          leds.setHSV(l-1, 240, 1, .1);
                          leds.setHSV(l-2, 240, 1, .1);
                          leds.setHSV(l-3, 240, 1, .1);

                          leds.setHSV(k, 240, 1, .1);
                          leds.setHSV(k+1, 240, 1, .1);
                          leds.setHSV(k+2, 240, 1, .1);
                          leds.setHSV(k+3, 240, 1, .1);

                          leds.setHSV(j, 240, 1, .1);
                          leds.setHSV(j+1, 240, 1, .1);
                          leds.setHSV(j+2, 240, 1, .1);
                          leds.setHSV(j+3, 240, 1, .1);

                          i-=4;
                          l-=4;
                          k+=4;
                          j+=4;

                        }
                        else
                        {
                          leds.setHSV(i, 240, 1, .1);
                          leds.setHSV(l, 240, 1, .1);
                          leds.setHSV(k, 240, 1, .1);
                          leds.setHSV(j, 240, 1, .1);
                          i-=1;
                          l-=1;
                          k+=1;
                          j+=1;
                        }

                        if (i < back_right_beam_end)
                        {
                          leds.setRGB(i+4, 0, 0, 0);
                          leds.setRGB(l+4, 0, 0, 0);
                          leds.setRGB(k-4, 0, 0, 0);
                          leds.setRGB(j-4, 0, 0, 0);
                        }

                        leds.flush();
                        // Microseconds
                        usleep(100000);

                        if (i == back_right_beam_start)
                        {
                          leds.setRGB(i, 0, 0, 0);
                          leds.setRGB(i+1, 0, 0, 0);
                          leds.setRGB(i+2, 0, 0, 0);
                          leds.setRGB(i+3, 0, 0, 0);

                          leds.setRGB(l, 0, 0, 0);
                          leds.setRGB(l+1, 0, 0, 0);
                          leds.setRGB(l+2, 0, 0, 0);
                          leds.setRGB(l+3, 0, 0, 0);

                          leds.setRGB(k, 0, 0, 0);
                          leds.setRGB(k-1, 0, 0, 0);
                          leds.setRGB(k-2, 0, 0, 0);
                          leds.setRGB(k-3, 0, 0, 0);

                          leds.setRGB(j, 0, 0, 0);
                          leds.setRGB(j-1, 0, 0, 0);
                          leds.setRGB(j-2, 0, 0, 0);
                          leds.setRGB(j-3, 0, 0, 0);

                          i-=1;
                          l-=1;
                          k+=1;
                          j+=1;

                          leds.flush();
                          // Microseconds
                          usleep(100000);
                        }
                      }
                    }
                    break;
                  }
        }

        // Successful Execution Logic
        if(success || timeout)
        {
          ROS_INFO("%s: Succeeded", action_name_.c_str());
          as_.setSucceeded();
          timeout_timer.stop();
          leds.clear();
          ROS_INFO("Cleared LED Strip");
          check_camera_status();
          timeout = false;
        }
      }
    }
    catch(const serial::SerialException &e)
    {
      ROS_ERROR("Action execution failed, unable to write to microcontroller,");
      ROS_ERROR("Ensure led microcontroller is connected.");
      ROS_ERROR("Attempting to reconnect to led microcontroller.");

      timeout_timer.stop();

      leds.connect(serial_port, 115200);
      // Need to sleep at least 2 seconds to wait for connection to be established
      sleep(3);
      leds.setLEDCount(led_count);

      ROS_INFO("Reconnected to LED strip, ready to control LED strip with %d leds.", led_count);

      as_.setPreempted();
      leds.clear();
      ROS_INFO("Cleared LED Strip");
      check_camera_status();
      success = false;
      timeout = false;
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
  try
  {
    leds.clear();
    ROS_INFO("Cleared LED Strip");
    check_camera_status();
    return true;
  }
  catch(const serial::SerialException &e)
  {
    service_reconnect();
  }
}

bool test_strip(bwi_led::test_strip::Request  &req,
              bwi_led::test_strip::Response &res)
{
  try
  {
    ROS_INFO("Testing LED Strip, will take about 30 seconds to complete.");

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
    ROS_INFO("Tested Colors on LED strip");

    return true;
  }
  catch(const serial::SerialException &e)
  {
    service_reconnect();
  }
}

bool set_first_five(bwi_led::set_first_five::Request  &req,
                    bwi_led::set_first_five::Response &res)
{
  try
  {
    leds.clear();
    sleep(1);

    leds.setHSV(0, 240, 1, .1);
    leds.setHSV(1, 120, 1, .1);
    leds.setHSV(2, 240, 1, .1);
    leds.setHSV(3, 120, 1, .1);
    leds.setHSV(4, 240, 1, .1);

    leds.flush();
    sleep(1);

    ROS_INFO("Set first five LEDs on strip");

    return true;
  }
  catch(const serial::SerialException &e)
  {
    service_reconnect();
  }
}

bool set_every_fifth(bwi_led::set_every_fifth::Request  &req,
                     bwi_led::set_every_fifth::Response &res)
{
  try
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

    ROS_INFO("Set every fifth LED on strip");

    return true;
  }
  catch(const serial::SerialException &e)
  {
    service_reconnect();
  }
}

bool set_camera_on(bwi_led::set_camera_on::Request  &req,
                   bwi_led::set_camera_on::Response &res)
{
  try
  {
    camera_on = true;
    check_camera_status();

    return true;
  }
  catch(const serial::SerialException &e)
  {
    service_reconnect();
  }
}

bool set_camera_off(bwi_led::set_camera_off::Request  &req,
                    bwi_led::set_camera_off::Response &res)
{
  try
  {
    camera_on = false;
    leds.clear();
    ROS_INFO("Set camera indicator off and cleared LED strip");
    return true;
  }
  catch(const serial::SerialException &e)
  {
    service_reconnect();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "led_control_server");
  ros::NodeHandle n;

  // Override the default ros sigint handler.
  // This must be set after the first NodeHandle is created.
  signal(SIGINT, ledSigintHandler);

  ros::NodeHandle privateNode("~");

  // Node Parameters
  // LED and Microcontroller Initialization 
  privateNode.param<int>("led_count",led_count,60);
  privateNode.param<string>("serial_port",serial_port,"/dev/metromini");

  // LED Segments
  privateNode.param<int>("camera_indicator_start",camera_indicator_start,29);
  privateNode.param<int>("camera_indicator_end",camera_indicator_end,30);

  privateNode.param<int>("back_center_start",back_center_start,85);
  privateNode.param<int>("back_center_end",back_center_end,94);

  // For center segments, start is in the center and end is the left/right most led of the segment
  privateNode.param<int>("front_center_left_start",front_center_left_start,29);
  privateNode.param<int>("front_center_left_end",front_center_left_end,34);

  privateNode.param<int>("front_center_right_start",front_center_right_start,30);
  privateNode.param<int>("front_center_right_end",front_center_right_end,25);

  privateNode.param<int>("back_center_left_start",back_center_left_start,90);
  privateNode.param<int>("back_center_left_end",back_center_left_end,85);

  privateNode.param<int>("back_center_right_start",back_center_right_start,89);
  privateNode.param<int>("back_center_right_end",back_center_right_end,94);

  // For beam segments, start is bottom most led and end is top most led of the segment
  privateNode.param<int>("front_left_beam_start",front_left_beam_start,60);
  privateNode.param<int>("front_left_beam_end",front_left_beam_end,73);

  privateNode.param<int>("front_right_beam_start",front_right_beam_start,119);
  privateNode.param<int>("front_right_beam_end",front_right_beam_end,106);

  privateNode.param<int>("back_left_beam_start",back_left_beam_start,59);
  privateNode.param<int>("back_left_beam_end",back_left_beam_end,46);

  privateNode.param<int>("back_right_beam_start",back_right_beam_start,0);
  privateNode.param<int>("back_right_beam_end",back_right_beam_end,13);

  // Initial connection to microcontroller
  leds.connect(serial_port, 115200);
  // Need to sleep at least 2 seconds to wait for connection to be established
  sleep(3);
  leds.setLEDCount(led_count);

  leds.clear();
  sleep(1);

  // Service Server advertisers
  ros::ServiceServer clear_service = n.advertiseService(ros::this_node::getName() + "/led_clear", clear_strip);
  ros::ServiceServer test_strip_service = n.advertiseService(ros::this_node::getName() + "/test_strip", test_strip);
  ros::ServiceServer set_first_five_service = n.advertiseService(ros::this_node::getName() + "/set_first_five", set_first_five);
  ros::ServiceServer set_every_fifth_service = n.advertiseService(ros::this_node::getName() + "/set_every_fifth", set_every_fifth);
  ros::ServiceServer set_camera_on_service = n.advertiseService(ros::this_node::getName() + "/set_camera_on", set_camera_on);
  ros::ServiceServer set_camera_off_service = n.advertiseService(ros::this_node::getName() + "/set_camera_off", set_camera_off);

  // Action Server advertisers
  LEDAction led_control_server(ros::this_node::getName());

  std::cout << ros::this_node::getName() << endl;

  ROS_INFO("Ready to control LED strip with %d leds.", led_count);
  ros::spin();

  return 0;
}