#include "ChangeFloor.h"

#include <boost/foreach.hpp>

#include "ros/console.h"
#include "ros/ros.h"

#include <actionlib/client/simple_action_client.h>
#include <bwi_msgs/LogicalActionAction.h>
#include <actasp/AspAtom.h>

namespace bwi_krexec {

ChangeFloor::ChangeFloor(const std::string &dest_room, knowledge_rep::LongTermMemoryConduit &ltmc) : dest_room(dest_room),
             done(false),
             asked(false),
             failed(false), LogicalNavigation("change_floor", ltmc) {}

             /*
void ChangeFloor::run() {
  
  if(!asked) {

    // TODO: Get the doors for this elevator.
    std::string dest_floor;


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
        //TODO: Get the door we're facing

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
          //TODO: Update the robot's state in the knowledgebase via sensestate?

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
 
}*/

actasp::Action *ChangeFloor::cloneAndInit(const actasp::AspFluent & fluent) const {
  return nullptr;
}

std::vector<std::string> ChangeFloor::getParameters() const {
  return {dest_room};
}

std::vector<std::string> ChangeFloor::prepareGoalParameters() const {
  // Get the doors for this elevator.
  std::string facing_door;
  //TODO: Get the door we're facing

  return {dest_room, facing_door};
}


}
