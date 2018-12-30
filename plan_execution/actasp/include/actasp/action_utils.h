#pragma once

#include <actasp/Action.h>
#include <actasp/AnswerSet.h>

#include <list>
#include <utility>
#include <set>
#include <functional>
#include "AspRule.h"

namespace actasp {

std::list<AnswerSet> filterPlans(const std::list<AnswerSet> &unfiltered_plans, const std::set<std::string>& allActions);

struct ActionMapDeepCopy {

  std::pair<std::string, Action *> operator()(const std::pair<std::string, Action *> &other) {

    return std::make_pair(other.first, other.second->clone());
  }
};

struct ActionMapDelete {

  void operator()(const std::pair<std::string, Action *> &other) {
    delete other.second;
  }
};

struct IsAnAction : public std::unary_function<const AspFluent &, bool> {

  IsAnAction(const std::set<std::string> &actions): actionNames(actions){};

  bool operator()(const AspFluent &fluent) const;

  std::set<std::string> actionNames;
};

AspFluent with_timestep(const AspFluent &fluent, uint32_t timestep);

AspFluent with_timestep(const AspFluent &fluent, Variable timestep);

AspRule add_timestep(const AspRule &fluent, Variable timestep);

AspRule add_timestep(const AspRule &fluent, uint32_t timestep);

AnswerSet planToAnswerSet(const std::list<std::unique_ptr<Action>> &plan);

// DEPRECATED
ActionSet actionMapToSet(const std::map<std::string, Action *> &actionMap);

ActionSet actionMapToSet(const std::map<std::string, ActionFactory> &actionMap);

}



