#ifndef bwi_actexec_GoThrough_h__guard
#define bwi_actexec_GoThrough_h__guard

#include "LogicalNavigation.h"

namespace bwi_krexec {

  
class GoThrough : public LogicalNavigation {
public:  
  explicit GoThrough(const std::string& doorName);
  
  void run();
  
  bool hasFailed() const {return failed;}
  
    Action *cloneAndInit(const actasp::AspFluent & fluent) const {
    return new GoThrough(fluent.getParameters().at(0));
  }
  
  virtual Action *clone() const {return new GoThrough(*this);}
    
private:
  bool failed;

};  
}

#endif
