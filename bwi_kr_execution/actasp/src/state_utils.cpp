
#include <actasp/state_utils.h>

namespace actasp {

  bool stateEquals::operator()(const std::set<AspFluent>& otherstate) const {

    if (state.size() != otherstate.size()) {
      return false;
    }
    std::set<AspFluent>::const_iterator thisIt = state.begin();
    std::set<AspFluent>::const_iterator otherIt = otherstate.begin();
    for(; thisIt!=state.end(); ++thisIt) {
      std::string thisstring = thisIt->toString(0);
      std::string otherstring = otherIt->toString(0);
      if (thisstring.compare(otherstring)!=0) { //different
        return false;
      }
      ++otherIt;
    }
    return true; //at this point..
  }
  
}

//this class became a template, this file is here for a syntax check

// using namespace std;
// 
// namespace actasp {

  

// bool  StateComparator::operator()(const std::set<AspFluent> &first,const std::set<AspFluent> &second) const {
//   if (first.size() != second.size())
//     return first.size() < second.size();
// 
//   //they have the same number of fluents
//   set<AspFluent>::const_iterator thisV = first.begin();
//   set<AspFluent>::const_iterator otherV = second.begin();
// 
//   ActionComparator lessThen; //ignores the time steps!
// 
//   for (; thisV != first.end(); ++thisV, ++otherV) {
//     //this comparison is costly, so I'm using this unelegant expression to minimize the calls to it.
//     if (lessThen(*thisV , *otherV))
//       return true;
//     if (lessThen(*otherV , *thisV))
//       return false;
//   }
// 
//   //they are the same
//   return false;
// }
// 
// 
// bool  StateComparatorRef::operator()(const std::set<AspFluentRef> &first,const std::set<AspFluentRef> &second) const {
//   if (first.size() != second.size())
//     return first.size() < second.size();
// 
//   //they have the same number of fluents
//   set<AspFluentRef>::const_iterator thisV = first.begin();
//   set<AspFluentRef>::const_iterator otherV = second.begin();
// 
//   ActionComparator lessThen; //ignores the time steps!
// 
//   for (; thisV != first.end(); ++thisV, ++otherV) {
//     //this comparison is costly, so I'm using this unelegant expression to minimize the calls to it.
//     if (lessThen(*thisV , *otherV))
//       return true;
//     if (lessThen(*otherV , *thisV))
//       return false;
//   }
// 
//   //they are the same
//   return false;
// }


// }

