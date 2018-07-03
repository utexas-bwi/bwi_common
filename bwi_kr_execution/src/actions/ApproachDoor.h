#ifndef bwi_actexec_ApproachDoor_h__guard
#define bwi_actexec_ApproachDoor_h__guard

#include "LogicalNavigation.h"

namespace bwi_krexec {

  
class ApproachDoor : public LogicalNavigation {
public:  
  explicit ApproachDoor();
  
  void run();
  
  bool hasFailed() const {return failed;}
  
  Action *cloneAndInit(const actasp::AspFluent & fluent) const {
    auto action = new ApproachDoor();
    action->door_name = fluent.getParameters().at(0);
    return action;
  }
  
  virtual Action *clone() const {return new ApproachDoor(*this);}

  int paramNumber() const override {
    return 1;
  }

  std::vector<std::string> getParameters() const override {
    std::vector<std::string> parameters;
    parameters.push_back(door_name);
    return parameters;
  }

  std::vector<std::string> prepareGoalParameters() const override;
    
private:
  bool failed;
  std::string door_name;

};  
}

#endif
