#include "CallElevator.h"

#include <boost/foreach.hpp>
#include <ctime>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <time.h>

#include "ActionFactory.h"
#include "../StaticFacts.h"

#include "bwi_kr_execution/CurrentStateQuery.h"
#include "bwi_kr_execution/UpdateFluents.h"

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ros/console.h>
#include <ros/package.h>
#include <ros/ros.h>

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
#include "bwi_services/SpeakMessage.h"
#include <move_base/move_base.h>
#include <move_base_msgs/MoveBaseLogging.h>
#include <std_srvs/Empty.h>

namespace bwi_krexec {

CallElevator::CallElevator() :
            done(false),
            asked(false),
            failed(false) {}

struct IsFluentFacing {

 bool operator() (const bwi_kr_execution::AspFluent& fluent) {
   return fluent.name == "facing";
 }

};

void CallElevator::run() {
  ros::NodeHandle n;

  ros::ServiceClient speak_message_client = n.serviceClient<bwi_services::SpeakMessage>("/speak_message_service_node/speak_message");
  bwi_services::SpeakMessage speak_srv;

  ros::ServiceClient init_count_client = n.serviceClient<std_srvs::Empty>("move_base/init_replan_count_elevator");
  std_srvs::Empty init_count_srv;

  ros::ServiceClient get_count_client = n.serviceClient<move_base_msgs::MoveBaseLogging>("/move_base/log_replan_count_elevator");
  move_base_msgs::MoveBaseLogging get_count_srv;

  actionlib::SimpleActionClient<bwi_msgs::LEDControlAction> ac("led_control_server", true);
  ac.waitForServer();
  bwi_msgs::LEDControlGoal goal;


  // std::ofstream log_file;
  // std::string log_filename = ros::package::getPath("led_study") + "/data/" + "elevator_state.csv";

  if(!asked && !done) {
    init_count_client.call(init_count_srv);

    std::string direction_text = (going_up) ? "up" : "down";

    // Get the doors for this elevator.
    std::vector<std::string> doors;
    std::list<actasp::AspAtom> static_facts = StaticFacts::staticFacts();
    BOOST_FOREACH(const actasp::AspAtom fact, static_facts) {
      if (fact.getName() == "elevhasdoor") {
        std::vector<std::string> params = fact.getParameters();
        if (params[0] == elevator) {
          doors.push_back(params[1]);
        }
      }
    }

    if (doors.size() == 0) {
      ROS_ERROR_STREAM("Unable to retrieve doors for elevator " << elevator << ". Cannot complete action!");
      done = true;
      failed = true;
    } else {

      // Figure out which of the doors we're facing.
      ros::ServiceClient krClient = n.serviceClient<bwi_kr_execution::CurrentStateQuery> ( "current_state_query" );
      krClient.waitForExistence();

      bwi_kr_execution::CurrentStateQuery csq;

      krClient.call(csq);

      std::vector<bwi_kr_execution::AspFluent>::const_iterator facingDoorIt =
        find_if(csq.response.answer.fluents.begin(), csq.response.answer.fluents.end(), IsFluentFacing());

      if (facingDoorIt == csq.response.answer.fluents.end()) {
        ROS_ERROR("CallElevator: Unable to ascertain which elevator door the robot is facing!");
        done = true;
        failed = true;
      } else {
        facing_door = facingDoorIt->variables[0];
        if (std::find(doors.begin(), doors.end(), facing_door) == doors.end()) {
          ROS_ERROR("CallElevator: Unable to ascertain which elevator door the robot is facing!");
          done = true;
          failed = true;
        } else {
          // Make sure that this is one of the elevator doors.
          std::vector<std::string> door_is_open;
          door_is_open.push_back("Door is open");

          srand(time(NULL));

          randLED = rand()%2;
          randSpeech = rand()%2;

          if (randLED == 1) {

            // time_t now = time(0);
            // tm *gmtm = gmtime(&now);
            // log_file.open(log_filename, std::ios_base::app | std::ios_base::out);
            // // state,led,date,time
            // log_file << "start," << randLED << "," <<  randSpeech << "," << (1900 + gmtm->tm_year) << "-" << (1 + gmtm->tm_mon) << "-" << gmtm->tm_mday << "," << (1 + gmtm->tm_hour) << ":" << (1 + gmtm->tm_min) << ":" << (1 + gmtm->tm_sec) << std::endl;
            // log_file.close();

            if (direction_text == "up")
            {
              goal.type.led_animations = bwi_msgs::LEDAnimations::UP;
            }
            else
            {
              goal.type.led_animations = bwi_msgs::LEDAnimations::DOWN;
            }
            goal.timeout = ros::Duration(0);
            ac.sendGoal(goal);
            if (randSpeech)
            {
              speak_srv.request.message = "Could you call the elevator to go " + direction_text + ", and then let me know when the door in front of me opens?";
              speak_message_client.call(speak_srv);
            }
          }
          else{
            // time_t now = time(0);
            // tm *gmtm = gmtime(&now);
            // log_file.open(log_filename, std::ios_base::app | std::ios_base::out);
            // // state,led,date,time
            // log_file << "start," << randLED << "," <<  randSpeech << "," << (1900 + gmtm->tm_year) << "-" << (1 + gmtm->tm_mon) << "-" << gmtm->tm_mday << "," << (1 + gmtm->tm_hour) << ":" << (1 + gmtm->tm_min) << ":" << (1 + gmtm->tm_sec) << std::endl;
            // log_file.close();

            if (randSpeech)
            {
              speak_srv.request.message = "Could you call the elevator to go " + direction_text + ", and then let me know when the door in front of me opens?";
              speak_message_client.call(speak_srv);
            }
          }

          askToCallElevator.reset(new CallGUI("askToCallElevator",
                                              CallGUI::CHOICE_QUESTION,
                                              "Could you call the elevator to go " + direction_text +
                                              ", and then let me know when the door in front of me opens?",
                                              120.0f,
                                              door_is_open));
          askToCallElevator->run();
        }
      }
    }
    asked = true;
  } else if(!done) {
    if (askToCallElevator->hasFinished()) {
      // Check response to see it's not a timeout.
      int response_idx = askToCallElevator->getResponseIndex();
      if (response_idx == 0) {

        ros::NodeHandle n;
        ros::ServiceClient krClient = n.serviceClient<bwi_kr_execution::UpdateFluents> ( "update_fluents" );
        krClient.waitForExistence();
        bwi_kr_execution::UpdateFluents uf;
        bwi_kr_execution::AspFluent open_door;
        open_door.name = "open";
        open_door.variables.push_back(facing_door);
        uf.request.fluents.push_back(open_door);

        krClient.call(uf);

        // time_t now = time(0);
        // tm *gmtm = gmtime(&now);
        // log_file.open(log_filename, std::ios_base::app | std::ios_base::out);
        // get_count_client.call(get_count_srv);
        // // state,led,date,time
        // log_file << "end," << randLED << ","  <<  randSpeech << "," << (1900 + gmtm->tm_year) << "-" << (1 + gmtm->tm_mon) << "-" << gmtm->tm_mday << "," << (1 + gmtm->tm_hour) << ":" << (1 + gmtm->tm_min) << ":" << (1 + gmtm->tm_sec) <<  "," << get_count_srv.response.replan_count << "," << get_count_srv.response.recovery_count << std::endl;
        // log_file.close();

        ac.cancelAllGoals();
        if(randSpeech)
        {
          CallGUI thanks("thanks", CallGUI::DISPLAY,  "Thanks! Would you mind helping me inside the elevator as well?");
          thanks.run();
        }
      } else {
        // A door didn't open in the timeout specified.
        failed = true;
      }
      done = true;
    }
  }


}

bool CallElevator::hasFinished() const {
  return done;
}

bool CallElevator::hasFailed() const {
  return failed;
}

actasp::Action *CallElevator::cloneAndInit(const actasp::AspFluent & fluent) const {
  CallElevator *other = new CallElevator();
  other->elevator = fluent.getParameters().at(0);
  other->going_up = fluent.getParameters().at(1) == "up"; //throws an exception if the parameter doesn't exist
  return other;
}

std::vector<std::string> CallElevator::getParameters() const {
  std::vector<std::string> params;
  params.reserve(2);
  params.push_back(elevator);
  params.push_back(going_up? "up" : "down");
  return params;
}

//if you want the action to be available only in simulation, or only
//on the robot, use the constructor that also takes a boolean.
ActionFactory callElevator(new CallElevator(), false);

}
