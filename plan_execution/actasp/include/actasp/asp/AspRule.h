#pragma once

#include <vector>
#include <string>
#include <sstream>
#include <actasp/asp/AspElement.h>
#include <actasp/asp/AspAtom.h>

namespace actasp {

/**
 * @brief The simplest combination of atoms
 * Ex: i_am() <- i_think().
 */
struct AspRule: public AspElement {
  
  AspRule() : head(), body() {}
  virtual ~AspRule() = default;
  AspRule(std::vector<AspAtom> head, std::vector<AspAtom> body) : head(std::move(head)), body(std::move(body)) {}

  AspRule& operator<< (const AspAtom &fluent) noexcept {
    body.push_back(fluent);
    return *this;
  }
  
  bool operator== (const AspRule other) const noexcept {
    return this->head == other.head && this->body == other.body;
  }
  
  std::vector<AspAtom> head;
  std::vector<AspAtom> body;

};

inline AspRule operator<=(const std::vector<AspAtom> &head, const std::vector<AspAtom> &body) {
  return AspRule(head, body);
}
}


