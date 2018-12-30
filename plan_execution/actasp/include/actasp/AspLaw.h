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
    return this->head == other.head && this->body == other.body;
  }

  std::string to_string(){
    std::stringstream string;
    bool first = true;
    for (const auto &headAtom: head) {
      if (!first) {
        string << ", ";
      }
      string << headAtom.to_string();
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
      string << bodyAtom.to_string();
      first = false;
    }
    return string.str();
  }
  
  std::vector<AtomType> head;
  std::vector<AtomType> body;

};

}


