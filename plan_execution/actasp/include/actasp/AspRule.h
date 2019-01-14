#pragma once

#include <vector>
#include <string>
#include <sstream>

namespace actasp {

template<typename AtomType>
struct AspRule {
  
  AspRule() : head(), body() {}
  AspRule(std::vector<AtomType> head, std::vector<AtomType> body) : head(head), body(body) {}

  AspRule& operator<< (AtomType fluent) noexcept {
    body.push_back(fluent);
    return *this;
  }
  
  bool operator== (const AspRule<AtomType> other) const noexcept {
    return this->head == other.head && this->body == other.body;
  }

  std::string to_string() const{
    std::stringstream sstream;
    bool first = true;
    for (const auto &headAtom: head) {
      if (!first) {
        sstream << ", ";
      }
      sstream << headAtom.to_string();
      first = false;
    }
    if (!body.empty()) {
      sstream << " :- ";
    }
    first = true;
    for (const auto &bodyAtom: body) {
      if (!first) {
        sstream << ", ";
      }
      sstream << bodyAtom.to_string();
      first = false;
    }
    sstream << ".";
    return sstream.str();
  }
  
  std::vector<AtomType> head;
  std::vector<AtomType> body;

};

template<typename AtomType>
AspRule<AtomType> operator<=(const std::vector<AtomType> head, const std::vector<AtomType> body) {
  return AspRule<AtomType>(head, body);
}
class AspFluent;
typedef AspRule<AspFluent> AspFluentRule;
}


