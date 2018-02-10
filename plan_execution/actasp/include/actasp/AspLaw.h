#ifndef actasp_AspLaw_h__guard
#define actasp_AspLaw_h__guard

#include <vector>
#include <string>

namespace actasp {

template<typename AtomType>
struct AspLaw {
  
  AspLaw() : head(), body() {}
  
  
  AspLaw& operator<< (AtomType fluent) throw (){
    body.push_back(fluent);
    return *this;
  }
  
  bool operator== (const AspLaw<AtomType> other) const throw () {
    if (other.head.size() != this->head.size())
      return false;
    if (other.body.size() != this->body.size())
      return false;
    typename std::vector<AtomType>::const_iterator otherHead = other.head.begin();
    typename std::vector<AtomType>::const_iterator thisHead = this->head.begin();
    for (; otherHead!=other.head.end(); ++otherHead) {
      std::string otherString = otherHead->toString(0);
      std::string thisString = thisHead->toString(0);
      if (otherString.compare(thisString)!=0)
        return false;
      ++thisHead;
    }

    typename std::vector<AtomType>::const_iterator otherBody = other.body.begin();
    typename std::vector<AtomType>::const_iterator thisBody = this->body.begin();
    for (; otherBody!=other.body.end(); ++otherBody) {
      std::string otherString = otherBody->toString(0);
      std::string thisString = thisBody->toString(0);
      if (otherString.compare(thisString)!=0)
        return false;
      ++thisBody;
    }

    //at this point:
    return true; 
  }
  
  std::vector<AtomType> head;
  std::vector<AtomType> body;

};

}

#endif
