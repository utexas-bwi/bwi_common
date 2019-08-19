#pragma once

#include <list>
#include <actasp/asp/AspFluent.h>

namespace actasp {

class AspFluent;

struct LexComparator {
  bool operator()(const std::list<AspFluentRef>& a, const std::list<AspFluentRef> &b) const;
  bool operator()(const std::list<AspFluent>& a, const std::list<AspFluent> &b) const;
};

}


