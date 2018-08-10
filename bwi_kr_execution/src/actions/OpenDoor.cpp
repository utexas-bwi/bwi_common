#include "OpenDoor.h"
#include "CallGUI.h"
#include "LogicalNavigation.h"
#include "actasp/AspFluent.h"

#include "plan_execution/CurrentStateQuery.h"
#include "plan_execution/AspRule.h"
#include "plan_execution/AspFluent.h"
#include "SenseLocation.h"

#include <ros/ros.h>

using namespace std;

namespace bwi_krexec {

OpenDoor::OpenDoor(const std::string &door_name, knowledge_rep::LongTermMemoryConduit &ltmc) :
            door(door_name),
            done(false),
            asked(false),
            open(false),
            failed(false),
            startTime(), ltmc(ltmc){}

  
void OpenDoor::run() {
  if(!asked) {
    CallGUI askToOpen("askToOpen", CallGUI::DISPLAY,  "Can you open door " + door + ", please?");
    askToOpen.run();
    asked = true;
    startTime = ros::Time::now();
    vector<string> params;
    params.push_back(door);
    senseDoor = unique_ptr<SenseLocation>(new SenseLocation(ltmc));
  }
  
  if(!open) {

    senseDoor->run();
    
    // TODO: check if door is open
    // Logical nav may advertise this as a service now
    
    if(!open && (ros::Time::now() - startTime) > ros::Duration(15.0)) {
      failed = true;
      done = true;
    }
    
    ROS_DEBUG_STREAM( "door open: " << open );
  }
  
  if(open) {
    CallGUI askToOpen("thank", CallGUI::DISPLAY,  "Thanks!");
    askToOpen.run();
    done = true;
  }

}  
  
actasp::Action* OpenDoor::cloneAndInit(const actasp::AspFluent& fluent) const {
  return nullptr;
}

std::vector<std::string> OpenDoor::getParameters() const {
  return {door};
}

}
