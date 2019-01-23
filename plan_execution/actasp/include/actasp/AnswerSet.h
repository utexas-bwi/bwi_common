#pragma once

#include <actasp/asp/AspFluent.h>

#include <string>
#include <set>
#include <list>
#include <map>
#include <vector>
#include <algorithm>
#include <stdexcept>
#include <actasp/Action.h>

namespace actasp {

class Action;
//TODO: Templatize to allow for nonfluent version
class AnswerSet {

public:

  AnswerSet() : satisfied(false), fluents() {}

  template<typename Iterator>
  AnswerSet(Iterator from, Iterator to) noexcept : satisfied(true), atoms(from, to) {
    for (const auto &atom: atoms) {
      try {
        AspFluent as_fluent{atom.getName(), atom.getArguments()};
        fluents.push_back(as_fluent);
      } catch (const std::invalid_argument &e) {
        continue;
      }
    }
    std::sort(fluents.begin(), fluents.end(), TimeStepComparator());
  }

  typedef std::vector<actasp::AspFluent> FluentSet;

  bool isSatisfied() const noexcept;

  bool contains(const actasp::AspAtom &fluent) const noexcept;
  bool contains(const actasp::AspFluent &fluent) const noexcept;

  std::list<std::unique_ptr<Action>> instantiateActions(const std::map<std::string, actasp::ActionFactory> &actionMap,
                                                        actasp::ResourceManager &resourceManager) const noexcept(false);

  // DEPRECATED
  std::list<std::unique_ptr<Action>>
  instantiateActions(const std::map<std::string, actasp::Action *> &actionMap) const noexcept(false);

  const FluentSet &getFluents() const noexcept {
    return fluents;
  }

  const std::vector<AspAtom> &getAtoms() const noexcept { return atoms; }

  std::set<actasp::AspFluent> getFluentsAtTime(uint32_t timeStep) const noexcept;

  uint32_t maxTimeStep() const noexcept(false);

private:

  bool satisfied;
  std::vector<AspAtom> atoms;
  FluentSet fluents;
};

typedef std::reference_wrapper<AnswerSet> AnswerSetRef;
}

