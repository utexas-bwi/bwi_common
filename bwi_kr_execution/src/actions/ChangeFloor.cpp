#include "ChangeFloor.h"

#include <boost/foreach.hpp>

#include "ActionFactory.h"
#include "../StaticFacts.h"

#include "bwi_kr_execution/UpdateFluents.h"
#include "ros/console.h"
#include "ros/ros.h"

#include "bwi_kr_execution/CurrentStateQuery.h"
#include "../msgs_utils.h"
#include "actasp/AnswerSet.h"

#include <actionlib/client/simple_action_client.h>
#include <bwi_msgs/LogicalNavigationAction.h>

/*******************************************************
*                   Service Headers                    *
********************************************************/
#include "bwi_services/SpeakMessage.h"

namespace bwi_krexec {

ChangeFloor::ChangeFloor() :
              done(false),
              asked(false),
              failed(false) {}

void ChangeFloor::run() {

  if(!asked) {

    ros::NodeHandle n;
    ros::ServiceClient speak_message_client = n.serviceClient<bwi_services::SpeakMessage>("/speak_message_service_node/speak_message");
    bwi_services::SpeakMessage speak_srv;

    // Get the doors for this elevator.
    std::string dest_floor;
    std::list<actasp::AspAtom> static_facts = StaticFacts::staticFacts();
    BOOST_FOREACH(const actasp::AspAtom fact, static_facts) {
      if (fact.getName() == "floor") {
        std::vector<std::string> params = fact.getParameters();
        if (params[0] == dest_room) {
          dest_floor = params[1];
          break;
        }
      }
    }

    if (dest_floor.empty()) {
      ROS_ERROR_STREAM("Unable to retrieve floor for destination " << dest_room << ". Cannot complete action!");
      done = true;
      failed = true;
    } else {
      std::vector<std::string> options;
      options.push_back("Reached!");
      askToChangeFloor.reset(new CallGUI("askToChangeFloor",
                                         CallGUI::DISPLAY,
                                         "Could you press the button for floor " + dest_floor +
                                         ", and then let me know when the elevator arrives there?"));
      askToChangeFloor->run();

      speak_srv.request.message = "Could you press the button for floor " + dest_floor + ", and then let me know when the elevator arrives there?";
      speak_message_client.call(speak_srv);

      // Retrieve current state fluents
      ros::ServiceClient krClient = n.serviceClient<bwi_kr_execution::CurrentStateQuery> ( "current_state_query" );
      krClient.waitForExistence();

      bwi_kr_execution::CurrentStateQuery csq;
      krClient.call(csq);

      actasp::AnswerSet answer = TranslateAnswerSet()(csq.response.answer);

      // Get the elevator we are using
      std::string elev_room;
      std::set<actasp::AspFluent> fluents_at_zero = answer.getFluentsAtTime(0);
      BOOST_FOREACH(const actasp::AspFluent fluent, fluents_at_zero) {
        if (fluent.getName() == "at") {
          // ROS_INFO_STREAM("Elev Room: " << fluent.getParameters()[0]);
          elev_room = fluent.getParameters()[0];
          break;
        }
      }

      // Get the door for the elevator we are using
      std::string elev_door;
      std::list<actasp::AspAtom> static_facts = StaticFacts::staticFacts();
      BOOST_FOREACH(const actasp::AspAtom fact, static_facts) {
        if (fact.getName() == "hasdoor") {
          std::vector<std::string> params = fact.getParameters();
          if (params[0] == elev_room) {
            // ROS_INFO_STREAM("Elev Door: " << params[1]);
            elev_door = params[1];
            break;
          }
        }
      }

      // Make robot face elevator door
      boost::shared_ptr<actionlib::SimpleActionClient<bwi_msgs::LogicalNavigationAction> > lnac;
      lnac.reset(new actionlib::SimpleActionClient<bwi_msgs::LogicalNavigationAction>("execute_logical_goal",
                                                                                                       true));
      lnac->waitForServer();
      bwi_msgs::LogicalNavigationGoal goal;
      goal.command.name = "approach";
      goal.command.value.push_back(elev_door);
      lnac->sendGoal(goal);
      //lnac->waitForResult();

      askToChangeFloor.reset(new CallGUI("askToChangeFloor",
                                         CallGUI::CHOICE_QUESTION,
                                         "Could you press the button for floor " + dest_floor +
                                           ", and then let me know when the elevator arrives there?",
                                         120.0f,
                                         options));
      askToChangeFloor->run();
    }
    asked = true;
  } else if(!done) {
    if (askToChangeFloor->hasFinished()) {
      // Check response to see it's positive.
      int response_idx = askToChangeFloor->getResponseIndex();
      if (response_idx == 0) {

        // Get the doors for this elevator.
        std::string facing_door;
        std::list<actasp::AspAtom> static_facts = StaticFacts::staticFacts();
        BOOST_FOREACH(const actasp::AspAtom fact, static_facts) {
          if (fact.getName() == "hasdoor") {
            std::vector<std::string> params = fact.getParameters();
            if (params[0] == dest_room) {
              // NOTE: This makes the assumption that an elevator room only has a single door, which is true for GDC.
              facing_door = params[1];
              break;
            }
          }
        }

        if (facing_door.empty()) {
          ROS_ERROR_STREAM("Unable to retrieve door we're facing for destination " << dest_room << ". Cannot complete action!");
          failed = true;
        } else {

          // Attempt to change the robot's location to this floor and location.
          boost::shared_ptr<actionlib::SimpleActionClient<bwi_msgs::LogicalNavigationAction> > lnac;
          lnac.reset(new actionlib::SimpleActionClient<bwi_msgs::LogicalNavigationAction>("execute_logical_goal",
                                                                                                           true));
          lnac->waitForServer();
          bwi_msgs::LogicalNavigationGoal goal;
          goal.command.name = "changefloor";
          goal.command.value.push_back(dest_room);
          goal.command.value.push_back(facing_door);
          lnac->sendGoal(goal);
          lnac->waitForResult();

          // TODO incorporate the return of the changefloor command as well.
          ros::NodeHandle n;
          ros::ServiceClient krClient = n.serviceClient<bwi_kr_execution::UpdateFluents> ( "update_fluents" );
          krClient.waitForExistence();
          bwi_kr_execution::UpdateFluents uf;

          bwi_kr_execution::AspFluent open_door;
          open_door.name = "open";
          open_door.variables.push_back(facing_door);

          bwi_kr_execution::AspFluent face_door;
          face_door.name = "facing";
          face_door.variables.push_back(facing_door);

          bwi_kr_execution::AspFluent beside_door;
          beside_door.name = "beside";
          beside_door.variables.push_back(facing_door);

          bwi_kr_execution::AspFluent at_loc;
          at_loc.name = "at";
          at_loc.variables.push_back(dest_room);

          uf.request.fluents.push_back(open_door);
          uf.request.fluents.push_back(face_door);
          uf.request.fluents.push_back(beside_door);
          uf.request.fluents.push_back(at_loc);
          krClient.call(uf);

          CallGUI thanks("thanks", CallGUI::DISPLAY,  "Thanks! Could you keep the door open while I exit the elevator?");
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

bool ChangeFloor::hasFinished() const {
  return done;
}

bool ChangeFloor::hasFailed() const {
  return failed;
}

actasp::Action *ChangeFloor::cloneAndInit(const actasp::AspFluent & fluent) const {
  ChangeFloor *other = new ChangeFloor();
  other->dest_room = fluent.getParameters().at(0);
  return other;
}

std::vector<std::string> ChangeFloor::getParameters() const {
  return std::vector<std::string>(1,dest_room);
}

//if you want the action to be available only in simulation, or only
//on the robot, use the constructor that also takes a boolean.
ActionFactory changeFloor(new ChangeFloor(), false);

}
