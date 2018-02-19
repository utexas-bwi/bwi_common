#include "ChangeFloor.h"

#include <boost/foreach.hpp>

#include "ActionFactory.h"
#include "plan_execution/StaticFacts.h"

#include "plan_execution/UpdateFluents.h"
#include "ros/console.h"
#include "ros/ros.h"

#include <actionlib/client/simple_action_client.h>
#include <bwi_msgs/LogicalActionAction.h>

namespace bwi_krexec {

ChangeFloor::ChangeFloor() :
             done(false),
             asked(false),
             failed(false) {}

void ChangeFloor::run() {
  
  if(!asked) {

    // Get the doors for this elevator.
    std::string dest_floor;
    std::list<actasp::AspAtom> static_facts = plan_exec::StaticFacts::staticFacts(); 
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
        std::list<actasp::AspAtom> static_facts = plan_exec::StaticFacts::staticFacts(); 
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
          boost::shared_ptr<actionlib::SimpleActionClient<bwi_msgs::LogicalActionAction> > lnac;
          lnac.reset(new actionlib::SimpleActionClient<bwi_msgs::LogicalActionAction>("execute_logical_goal",
                                                                                                           true));
          lnac->waitForServer();
          bwi_msgs::LogicalActionGoal goal;
          goal.command.name = "changefloor";
          goal.command.value.push_back(dest_room);
          goal.command.value.push_back(facing_door);
          lnac->sendGoal(goal);
          lnac->waitForResult();

          // TODO incorporate the return of the changefloor command as well.
          ros::NodeHandle n;
          ros::ServiceClient krClient = n.serviceClient<plan_execution::UpdateFluents> ( "update_fluents" );
          krClient.waitForExistence();
          plan_execution::UpdateFluents uf;

          plan_execution::AspFluent open_door;
          open_door.name = "open";
          open_door.variables.push_back(facing_door);

          plan_execution::AspFluent face_door;
          face_door.name = "facing";
          face_door.variables.push_back(facing_door);

          plan_execution::AspFluent beside_door;
          beside_door.name = "beside";
          beside_door.variables.push_back(facing_door);

          plan_execution::AspFluent at_loc;
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
