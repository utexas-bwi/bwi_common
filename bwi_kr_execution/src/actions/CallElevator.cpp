#include "CallElevator.h"

#include <boost/foreach.hpp>

#include "ActionFactory.h"
#include "../StaticFacts.h"

#include "bwi_kr_execution/UpdateFluents.h"
#include "ros/console.h"
#include "ros/ros.h"

namespace bwi_krexec {

CallElevator::CallElevator() :
            done(false),
            asked(false),
            failed(false) {}

void CallElevator::run() {
  
  if(!asked) {
    std::string direction_text = (going_up) ? "up" : "down";

    // Get the doors for this elevator.
    doors.clear();
    std::list<actasp::AspAtom> static_facts = StaticFacts::staticFacts(); 
    BOOST_FOREACH(const actasp::AspAtom fact, static_facts) {
      std::cout << fact.toString() << std::endl;
      if (fact.getName() == "elevhasdoor") {
        std::vector<std::string> params = fact.getParameters();
        if (params[0] == elevator) {
          /* std::cout << "  " << params[1] << std::endl; */
          doors.push_back(params[1]);
        }
      }
    }

    if (doors.size() == 0) {
      ROS_ERROR_STREAM("Unable to retrieve doors for elevator " << elevator << ". Cannot complete action!");
      done = true;
      failed = true;
    } else {
      askToCallElevator.reset(new CallGUI("askToCallElevator", 
                                          CallGUI::CHOICE_QUESTION,  
                                          "Could you call elevator " + elevator + " to go " + direction_text + 
                                            ", and then let me know which door opened?", 
                                          120.0f, 
                                          doors));
      askToCallElevator->run();
    }
    asked = true;
  } else if(!done) {
    if (askToCallElevator->hasFinished()) {
      // Check response to see it's positive.
      int response_idx = askToCallElevator->getResponseIndex();
      if (response_idx >= 0 && response_idx < doors.size()) {

        ros::NodeHandle n;
        ros::ServiceClient krClient = n.serviceClient<bwi_kr_execution::UpdateFluents> ( "update_fluents" );
        krClient.waitForExistence();
        bwi_kr_execution::UpdateFluents uf;
        bwi_kr_execution::AspFluent open_door;
        open_door.name = "open";
        open_door.variables.push_back(doors[response_idx]);
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
ActionFactory callElevator(new CallElevator());

}
