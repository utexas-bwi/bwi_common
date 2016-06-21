#ifndef bwi_actexec_GotoObject_h__guard
#define bwi_actexec_GotoObject_h__guard

#include "LogicalNavigation.h"

namespace bwi_krexec {

  
class GotoObject : public LogicalNavigation {
public:  
  explicit GotoObject(const std::string& objectName);
  
  void run();
  
  bool hasFailed() const {return failed;}
  
  Action *cloneAndInit(const actasp::AspFluent & fluent) const {
    return new GotoObject(fluent.getParameters().at(0));
  }
  
  virtual Action *clone() const {return new GotoObject(*this);}
    
private:
  bool failed;

};  
}

#endif
