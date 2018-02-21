#include "CallSimulatedElevator.h"

#include <boost/foreach.hpp>

#include "plan_execution/CurrentStateQuery.h"
#include "bwi_msgs/DoorHandlerInterface.h"

#include "ActionFactory.h"
#include "plan_execution/StaticFacts.h"

#include "plan_execution/UpdateFluents.h"

#include "ros/console.h"
#include "ros/ros.h"

namespace bwi_krexec {

CallSimulatedElevator::CallSimulatedElevator() :
            done(false),
            failed(false),
            requestSent(false) {}

void CallSimulatedElevator::run() {

  if (!requestSent) {

    // Get the doors for this elevator.
    doors.clear();
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

      ros::NodeHandle n;
      ros::ServiceClient doorClient = n.serviceClient<bwi_msgs::DoorHandlerInterface> ("/update_doors");
      doorClient.waitForExistence();

      bwi_msgs::DoorHandlerInterface dhi;

      selectedDoor = doors[rand() % doors.size()];
      dhi.request.all_doors = false;
      dhi.request.door = selectedDoor;
      dhi.request.open = true;
      dhi.request.open_timeout = 45.0f;

      doorClient.call(dhi);

      requestSent = true;
    }
  } else if (!done) {

    // The door may not be visible to the robot in simulation. Force state update.
    ros::NodeHandle n;
    ros::ServiceClient krClient = n.serviceClient<plan_execution::UpdateFluents> ( "update_fluents" );
    krClient.waitForExistence();
    plan_execution::UpdateFluents uf;
    plan_execution::AspFluent open_door;
    open_door.name = "open";
    open_door.variables.push_back(selectedDoor);
    uf.request.fluents.push_back(open_door);

    /* TODO: Remove this hack. Also, pretend you're facing the door that opened. This way it's shorter to use the open elevator. */
    plan_execution::AspFluent facing_door;
    facing_door.name = "facing";
    facing_door.variables.push_back(selectedDoor);
    uf.request.fluents.push_back(facing_door);

    krClient.call(uf);

    done = true;
  }

}

bool CallSimulatedElevator::hasFinished() const {
  return done;
}

bool CallSimulatedElevator::hasFailed() const {
  return failed;
}

actasp::Action *CallSimulatedElevator::cloneAndInit(const actasp::AspFluent & fluent) const {
  CallSimulatedElevator *other = new CallSimulatedElevator();
  other->elevator = fluent.getParameters().at(0);
  other->going_up = fluent.getParameters().at(1) == "up"; //throws an exception if the parameter doesn't exist
  return other;
}

std::vector<std::string> CallSimulatedElevator::getParameters() const {
  std::vector<std::string> params;
  params.reserve(2);
  params.push_back(elevator);
  params.push_back(going_up? "up" : "down");
  return params;
}

//if you want the action to be available only in simulation, or only
//on the robot, use the constructor that also takes a boolean.
ActionFactory callSimulatedElevator(new CallSimulatedElevator(), true);

}
