
#ifndef bwi_krexec_OpenSimulatedDoor_h__guard
#define bwi_krexec_OpenSimulatedDoor_h__guard

#include "actasp/Action.h"
#include "LogicalNavigation.h"

#include <string>

namespace bwi_krexec {

class OpenSimulatedDoor : public actasp::Action{
public:
  OpenSimulatedDoor(const std::string &door_name, knowledge_rep::LongTermMemoryConduit &ltmc);

  int paramNumber() const {return 1;}
  
  std::string getName() const {return "opendoor";}
  
  void run();
  
  bool hasFinished() const {return done;}
  
  actasp::Action *cloneAndInit(const actasp::AspFluent & fluent) const;
  
  virtual actasp::Action *clone() const {return nullptr;}
  
private:
  
 std::vector<std::string> getParameters() const;
 
 std::string door;
 bool done;
 bool requestSent;

 std::unique_ptr<LogicalNavigation> senseDoor;

 knowledge_rep::LongTermMemoryConduit &ltmc;
 
};

}
 
#endif
