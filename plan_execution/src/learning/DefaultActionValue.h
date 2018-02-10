#ifndef plan_exec_DefaultActionValue_h__guard
#define plan_exec_DefaultActionValue_h__guard

#include <actasp/actaspfwd.h>

namespace plan_exec {

struct DefaultActionValue {
  
  virtual double value(const actasp::AspFluent &action) = 0;
  
  virtual ~DefaultActionValue() {}
  
};

}


#endif
