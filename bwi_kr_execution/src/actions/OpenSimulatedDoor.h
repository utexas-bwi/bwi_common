
#ifndef bwi_krexec_OpenSimulatedDoor_h__guard
#define bwi_krexec_OpenSimulatedDoor_h__guard

#include "actasp/Action.h"
#include "plan_execution/LogicalAction.h"

#include <string>

namespace bwi_krexec {

class OpenSimulatedDoor : public actasp::Action{
public:
  OpenSimulatedDoor();

  int paramNumber() const {return 1;}
  
  std::string getName() const {return "opendoor";}
  
  void run();
  
  bool hasFinished() const {return done;}
  
  actasp::Action *cloneAndInit(const actasp::AspFluent & fluent) const;
  
  virtual actasp::Action *clone() const {return new OpenSimulatedDoor(*this);}
  
private:
  
 std::vector<std::string> getParameters() const;
 
 std::string door;
 bool done;
 bool requestSent;

 plan_exec::LogicalAction* senseDoor;
 
};

}
 
#endif
 