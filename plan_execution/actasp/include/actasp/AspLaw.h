#pragma once

#include <vector>
#include <string>
#include <sstream>

namespace actasp {

template<typename AtomType>
struct AspLaw {
  
  AspLaw() : head(), body() {}
  AspLaw(std::vector<AtomType> head, std::vector<AtomType> body) : head(head), body(body) {}

  AspLaw& operator<< (AtomType fluent) noexcept {
    body.push_back(fluent);
    return *this;
  }
  
  bool operator== (const AspLaw<AtomType> other) const noexcept {
    if (other.head.size() != this->head.size())
      return false;
    if (other.body.size() != this->body.size())
      return false;
    typename std::vector<AtomType>::const_iterator otherHead = other.head.begin();
    typename std::vector<AtomType>::const_iterator thisHead = this->head.begin();
    for (; otherHead!=other.head.end(); ++otherHead) {
      std::string otherString = otherHead->toString(0);
      std::string thisString = thisHead->toString(0);
      if (otherString != thisString)
        return false;
      ++thisHead;
    }

    typename std::vector<AtomType>::const_iterator otherBody = other.body.begin();
    typename std::vector<AtomType>::const_iterator thisBody = this->body.begin();
    for (; otherBody!=other.body.end(); ++otherBody) {
      std::string otherString = otherBody->toString(0);
      std::string thisString = thisBody->toString(0);
      if (otherString != thisString)
        return false;
      ++thisBody;
    }

    //at this point:
    return true; 
  }

  std::string toString(){
    std::stringstream string;
    bool first = true;
    for (const auto &headAtom: head) {
      if (!first) {
        string << ", ";
      }
      string << headAtom.toString();
      first = false;
    }
    if (!body.empty()) {
      string << " :- ";
    }
    first = true;
    for (const auto &bodyAtom: body) {
      if (!first) {
        string << ", ";
      }
      string << bodyAtom.toString();
      first = false;
    }
    return string.str();
  }
  
  std::vector<AtomType> head;
  std::vector<AtomType> body;

};

}


