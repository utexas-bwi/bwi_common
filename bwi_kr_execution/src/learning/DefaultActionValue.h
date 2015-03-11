#ifndef bwi_krexec_DefaultActionValue_h__guard
#define bwi_krexec_DefaultActionValue_h__guard

#include <actasp/actaspfwd.h>

namespace bwi_krexec {

struct DefaultActionValue {
  
  virtual double value(const actasp::AspFluent &action) = 0;
  
  virtual ~DefaultActionValue() {}
  
};

}


#endif
