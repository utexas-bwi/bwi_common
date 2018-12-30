#pragma once

#include <actasp/AspFluent.h>

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
  AnswerSet(Iterator from, Iterator to) noexcept : satisfied(true), fluents(from, to) {
    std::sort(fluents.begin(), fluents.end(), TimeStepComparator());
  }

  typedef std::vector<actasp::AspFluent> FluentSet;

  bool isSatisfied() const noexcept;

  bool contains(const actasp::AspFluent &fluent) const noexcept;

  std::list<std::unique_ptr<Action>> instantiateActions(const std::map<std::string, actasp::ActionFactory> &actionMap,
                                                        actasp::ResourceManager &resourceManager) const noexcept(false);

  // DEPRECATED
  std::list<std::unique_ptr<Action>>
  instantiateActions(const std::map<std::string, actasp::Action *> &actionMap) const noexcept(false);

  const FluentSet &getFluents() const noexcept { return fluents; }

  std::set<actasp::AspFluent> getFluentsAtTime(uint32_t timeStep) const noexcept;

  uint32_t maxTimeStep() const noexcept(false);

private:

  bool satisfied;
  FluentSet fluents;
};

typedef std::reference_wrapper<AnswerSet> AnswerSetRef;
}

