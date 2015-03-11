
#include <actasp/AspFluent.h>

#include "DefaultTimes.h"

#include <iostream>

using namespace actasp;

namespace bwi_krexec {
 
double DefaultTimes::value(const AspFluent& action) {
  
  std::cerr << "invoked on " << action.getName() << std::endl;
  
  if(action.getName() == "approach")
    return -1;
  
  return 0.;
}

  
}