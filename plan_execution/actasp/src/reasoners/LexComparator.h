#pragma once

#include <list>

namespace actasp {
  
class AspFluentRef;
class AspFluent;

struct LexComparator {
  bool operator()(const std::list<AspFluentRef>& a, const std::list<AspFluentRef> &b) const;
  bool operator()(const std::list<AspFluent>& a, const std::list<AspFluent> &b) const;
};

}


