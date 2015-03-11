#ifndef bwi_actexec_ApproachDoor_h__guard
#define bwi_actexec_ApproachDoor_h__guard

#include "LogicalNavigation.h"

namespace bwi_krexec {

  
class ApproachDoor : public LogicalNavigation {
public:  
  explicit ApproachDoor(const std::string& doorName);
  
  void run();
  
  bool hasFailed() const {return failed;}
  
  Action *cloneAndInit(const actasp::AspFluent & fluent) const {
    return new ApproachDoor(fluent.getParameters().at(0));
  }
  
  virtual Action *clone() const {return new ApproachDoor(*this);}
    
private:
  bool failed;

};  
}

#endif
