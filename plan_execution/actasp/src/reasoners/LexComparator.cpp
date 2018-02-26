#include "LexComparator.h"

#include <algorithm>

#include <actasp/AspFluent.h>

using namespace std;

namespace actasp {

bool LexComparator::operator()(const list<AspFluentRef>& a, const list<AspFluentRef> &b) const {
    return lexicographical_compare(a.begin(),a.end(),b.begin(),b.end(),ActionComparator());
}

bool LexComparator::operator()(const std::list<AspFluent>& a, const std::list<AspFluent> &b) const {
  return lexicographical_compare(a.begin(),a.end(),b.begin(),b.end(),ActionComparator());
}
  
}