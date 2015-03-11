#ifndef bwi_krexec_DefaultTimes_h__guard
#define bwi_krexec_DefaultTimes_h__guard

#include <actasp/actaspfwd.h>

#include "DefaultActionValue.h"

namespace bwi_krexec {

struct DefaultTimes : public DefaultActionValue {
  
  virtual double value(const actasp::AspFluent &action);
  
};

}


#endif