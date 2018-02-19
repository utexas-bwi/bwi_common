
#include <actasp/AspFluent.h>

#include "DefaultTimes.h"

#include <iostream>

using namespace actasp;

namespace plan_exec {
 
double DefaultTimes::value(const AspFluent& action) {
  
  if(action.getName() == "approach")
    return -1;
  
  return 0.;
}

  
}