#include "CallElevator.h"

#include <boost/foreach.hpp>

#include "ActionFactory.h"
#include "plan_execution/StaticFacts.h"

#include "plan_execution/CurrentStateQuery.h"
#include "plan_execution/UpdateFluents.h"
#include "ros/console.h"
#include "ros/ros.h"

namespace bwi_krexec {

CallElevator::CallElevator() :
            done(false),
            asked(false),
            failed(false) {}

struct IsFluentFacing {
 
 bool operator() (const plan_execution::AspFluent& fluent) {
   return fluent.name == "facing";
 }
 
};

void CallElevator::run() {
  
  if(!asked && !done) {
    std::string direction_text = (going_up) ? "up" : "down";

    // Get the doors for this elevator.
    std::vector<std::string> doors;
    std::list<actasp::AspAtom> static_facts = plan_exec::StaticFacts::staticFacts(); 
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
      ros::NodeHandle n;
      ros::ServiceClient krClient = n.serviceClient<plan_execution::CurrentStateQuery> ( "current_state_query" );
      krClient.waitForExistence();

      plan_execution::CurrentStateQuery csq;

      krClient.call(csq);
  
      std::vector<plan_execution::AspFluent>::const_iterator facingDoorIt = 
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
        ros::ServiceClient krClient = n.serviceClient<plan_execution::UpdateFluents> ( "update_fluents" );
        krClient.waitForExistence();
        plan_execution::UpdateFluents uf;
        plan_execution::AspFluent open_door;
        open_door.name = "open";
        open_door.variables.push_back(facing_door);
        uf.request.fluents.push_back(open_door);

        krClient.call(uf);

        CallGUI thanks("thanks", CallGUI::DISPLAY,  "Thanks! Would you mind helping me inside the elevator as well?");
        thanks.run();
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
