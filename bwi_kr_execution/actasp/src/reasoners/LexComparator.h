#ifndef actasp_LexComparator_h__guard
#define actasp_LexComparator_h__guard

#include <list>

namespace actasp {
  
class AspFluentRef;

struct LexComparator {
  bool operator()(const std::list<AspFluentRef>& a, const std::list<AspFluentRef> &b) const;
};

}

#endif
