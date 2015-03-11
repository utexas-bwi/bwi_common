#ifndef bwi_krexec_RewardFunction_h__guard
#define bwi_krexec_RewardFunction_h__guard

#include <actasp/AspFluent.h>

#include <set>

namespace bwi_krexec {

template <typename State>
struct RewardFunction {
  
  virtual double r(const State &initial, const actasp::AspFluent &action, const State &final) const throw() = 0; 
  
  virtual ~RewardFunction() {}  
};

}

#endif
