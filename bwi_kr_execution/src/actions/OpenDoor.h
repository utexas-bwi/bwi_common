
#ifndef bwi_krexec_OpenDoor_h__guard
#define bwi_krexec_OpenDoor_h__guard

#include "actasp/Action.h"
#include "plan_execution/LogicalAction.h"
#include <ros/ros.h>

#include <string>

namespace bwi_krexec {

class OpenDoor : public actasp::Action{
public:
  OpenDoor();

  int paramNumber() const {return 1;}
  
  std::string getName() const {return "opendoor";}
  
  void run();
  
  bool hasFinished() const {return done;}
  
  virtual bool hasFailed() const {return failed;}
  
  actasp::Action *cloneAndInit(const actasp::AspFluent & fluent) const;
  
  virtual actasp::Action *clone() const {return new OpenDoor(*this);}
  
private:
  
 std::vector<std::string> getParameters() const;
 
 std::string door;
 bool done;
 bool asked;
 bool open;
 bool failed;
 ros::Time startTime;

 plan_exec::LogicalAction* senseDoor;
 
};

}
 
#endif
 