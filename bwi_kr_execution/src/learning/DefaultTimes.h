#ifndef plan_exec_DefaultTimes_h__guard
#define plan_exec_DefaultTimes_h__guard

#include <actasp/actaspfwd.h>

#include "DefaultActionValue.h"

namespace plan_exec {

struct DefaultTimes : public DefaultActionValue {
  
  virtual double value(const actasp::AspFluent &action);
  
};

}


#endif