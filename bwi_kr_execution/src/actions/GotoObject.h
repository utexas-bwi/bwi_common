#ifndef bwi_actexec_GotoObject_h__guard
#define bwi_actexec_GotoObject_h__guard

#include "LogicalNavigation.h"

namespace bwi_krexec {

  
class GotoObject : public LogicalNavigation {
public:  
  explicit GotoObject();
  
  void run() override;
  
  bool hasFailed() const override {return failed;}
  
  Action *cloneAndInit(const actasp::AspFluent & fluent) const override {
      auto action = new GotoObject();
      action->location_id = std::stoi(fluent.getParameters().at(0));
      return action;
  }

  Action *clone() const override {return new GotoObject(*this);}

  int paramNumber() const override {
    return 1;
  }

  std::vector<std::string> getParameters() const override;

  std::vector<std::string> prepareGoalParameters() const override;
    
private:
  bool failed;
  int location_id;
};  
}

#endif
