#include "OpenDoor.h"

#include "CallGUI.h"
#include "LogicalNavigation.h"

#include "ActionFactory.h"

#include "actasp/AspFluent.h"

#include "bwi_kr_execution/CurrentStateQuery.h"
#include "bwi_kr_execution/AspRule.h"
#include "bwi_kr_execution/AspFluent.h"

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ros/package.h>
#include <ros/ros.h>

#include <ctime>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <time.h>

/*******************************************************
*                   segbot_led Headers                 *
********************************************************/
#include "bwi_msgs/LEDActionResult.h"
#include "bwi_msgs/LEDAnimations.h"
#include "bwi_msgs/LEDClear.h"
#include "bwi_msgs/LEDControlAction.h"

/*******************************************************
*                   Service Headers                    *
********************************************************/
//#include "bwi_services/SpeakMessage.h"
#include <move_base/move_base.h>
//#include <move_base_msgs/MoveBaseLogging.h>
#include <std_srvs/Empty.h>

using namespace std;

namespace bwi_krexec {

OpenDoor::OpenDoor() :
            door(),
            done(false),
            asked(false),
            open(false),
            failed(false),
            startTime(){}


void OpenDoor::run() {
  ros::NodeHandle n;

  //ros::ServiceClient speak_message_client = n.serviceClient<bwi_services::SpeakMessage>("/speak_message_service/speak_message");
  //bwi_services::SpeakMessage speak_srv;

  actionlib::SimpleActionClient<bwi_msgs::LEDControlAction> ac("led_control_server", true);
  ac.waitForServer();
  bwi_msgs::LEDControlGoal goal;

  ros::ServiceClient init_count_client = n.serviceClient<std_srvs::Empty>("move_base/init_replan_count_door");
  std_srvs::Empty init_count_srv;

//ros::ServiceClient get_count_client = n.serviceClient<move_base_msgs::MoveBaseLogging>("/move_base/log_replan_count_door");
  //move_base_msgs::MoveBaseLogging get_count_srv;

  // std::ofstream log_file;
  // std::string log_filename = ros::package::getPath("led_study") + "/data/" + "assist_state.csv";

  if(!asked) {
    init_count_client.call(init_count_srv);
    srand(time(NULL));

    randLED = rand()%2;
    randSpeech = rand()%2;

    if (randLED == 1) {

      // time_t now = time(0);
      // tm *gmtm = gmtime(&now);
      // log_file.open(log_filename, std::ios_base::app | std::ios_base::out);
      // // state,led,date,time
      // log_file << "start," << randLED <<  ","  <<  randSpeech << "," << (1900 + gmtm->tm_year) << "-" << (1 + gmtm->tm_mon) << "-" << gmtm->tm_mday << "," << (1 + gmtm->tm_hour) << ":" << (1 + gmtm->tm_min) << ":" << (1 + gmtm->tm_sec) << std::endl;
      // log_file.close();

      goal.type.led_animations = bwi_msgs::LEDAnimations::NEED_ASSIST;
      goal.timeout = ros::Duration(0);
      ac.sendGoal(goal);
      /*if(randSpeech)
      {
        speak_srv.request.message = "Can you open door " + door + ", please?";
        speak_message_client.call(speak_srv);
      }*/
    }
    else {
      // time_t now = time(0);
      // tm *gmtm = gmtime(&now);
      // log_file.open(log_filename, std::ios_base::app | std::ios_base::out);
      // // state,led,date,time
      // log_file << "start," << randLED << ","  <<  randSpeech << "," << (1900 + gmtm->tm_year) << "-" << (1 + gmtm->tm_mon) << "-" << gmtm->tm_mday << "," << (1 + gmtm->tm_hour) << ":" << (1 + gmtm->tm_min) << ":" << (1 + gmtm->tm_sec) << std::endl;
      // log_file.close();
      /*if(randSpeech)
      {
        speak_srv.request.message = "Can you open door " + door + ", please?";
        speak_message_client.call(speak_srv);
      }*/
    }

    CallGUI askToOpen("askToOpen", CallGUI::DISPLAY,  "Can you open door " + door + ", please?");
    askToOpen.run();
    asked = true;
    startTime = ros::Time::now();
  }

  if(!open) {
    vector<string> params;
    params.push_back(door);
    LogicalNavigation senseDoor("sensedoor",params);

    senseDoor.run();

    //check if door is open
    ros::ServiceClient currentClient = n.serviceClient<bwi_kr_execution::CurrentStateQuery> ( "current_state_query" );

    bwi_kr_execution::AspFluent openFluent;
    openFluent.name = "open";
    openFluent.timeStep = 0;
    openFluent.variables.push_back(door);

    bwi_kr_execution::AspRule rule;
    rule.head.push_back(openFluent);

    bwi_kr_execution::CurrentStateQuery csq;
    csq.request.query.push_back(rule);

    currentClient.call(csq);

    open = csq.response.answer.satisfied;

    // if(!open && (ros::Time::now() - startTime) > ros::Duration(15.0)) {
    if(!open && (ros::Time::now() - startTime) > ros::Duration(300.0)) {
      failed = true;
      done = true;
    }

    ROS_DEBUG_STREAM( "door open: " << open );
  }

  if(open) {
    ac.cancelAllGoals();

    // time_t now = time(0);
    // tm *gmtm = gmtime(&now);
    // log_file.open(log_filename, std::ios_base::app | std::ios_base::out);
    // get_count_client.call(get_count_srv);
    // // state,led,date,time
    // log_file << "end," << randLED << "," << randSpeech << "," << (1900 + gmtm->tm_year) << "-" << (1 + gmtm->tm_mon) << "-" << gmtm->tm_mday << "," << (1 + gmtm->tm_hour) << ":" << (1 + gmtm->tm_min) << ":" << (1 + gmtm->tm_sec) << "," << get_count_srv.response.replan_count << "," << get_count_srv.response.recovery_count << std::endl;
    // log_file.close();
    /*if(randSpeech)
    {
      speak_srv.request.message = "Thanks, Please keep the door open for me so I can pass through.";
      speak_message_client.call(speak_srv);
    }*/
    CallGUI askToOpen("thank", CallGUI::DISPLAY,  "Thanks, Please keep the door open for me so I can pass through.");
    askToOpen.run();
    done = true;
  }

}

actasp::Action* OpenDoor::cloneAndInit(const actasp::AspFluent& fluent) const {
  OpenDoor *newAction = new OpenDoor();
  newAction->door = fluent.getParameters().at(0);

  return newAction;
}

std::vector<std::string> OpenDoor::getParameters() const {
  vector<string> param;
  param.push_back(door);
  return param;
}


ActionFactory openDoorFactory(new OpenDoor(), false);

}
