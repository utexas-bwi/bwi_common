#include <utility>

#include <utility>

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

/**
 * A stable model of an ASP program, coupled with some awareness of which atoms in the model
 * are actually timestepped (AKA fluents)
 */
struct AnswerSet {
  AnswerSet() : satisfied(false) {

  }

  AnswerSet(std::vector<AspAtom> atoms, std::vector<AspFluent> fluents) : satisfied(true), atoms(std::move(atoms)),
                                                                          fluents(
                                                                              std::move(fluents)) {
    std::sort(fluents.begin(), fluents.end(), TimeStepComparator());
  }

  typedef std::vector<actasp::AspFluent> FluentSet;

  std::list<std::unique_ptr<Action>> instantiateActions(const std::map<std::string, actasp::ActionFactory> &actionMap,
                                                        actasp::ResourceManager &resourceManager) const noexcept(false);

  // DEPRECATED
  std::list<std::unique_ptr<Action>>
  instantiateActions(const std::map<std::string, actasp::Action *> &actionMap) const noexcept(false);

  std::set<actasp::AspFluent> getFluentsAtTime(uint32_t timeStep) const noexcept;

  uint32_t maxTimeStep() const noexcept(false);

  virtual std::string to_string() const;

  const bool satisfied;
  const std::vector<AspAtom> atoms;
  const FluentSet fluents;
};

typedef std::reference_wrapper<AnswerSet> AnswerSetRef;

/**
 * An answer set, but with some atoms specially noted as specifying actions
 */
struct Plan : public AnswerSet {
  Plan() : AnswerSet() {
  }

  Plan(std::vector<AspAtom> atoms, std::vector<AspFluent> fluents, std::vector<AspFluent> plan) : actions(std::move(plan)),
                                                                                                  AnswerSet(
                                                                                                      std::move(atoms),
                                                                                                      std::move(
                                                                                                          fluents)) {
  }

  /**
   * @brief A sequence of fluents starting at timestep 1 representing actions of the plan
   */
  const std::vector<AspFluent> actions;

  std::string to_string() const override;

};

typedef std::reference_wrapper<Plan> PlanRef;

}


