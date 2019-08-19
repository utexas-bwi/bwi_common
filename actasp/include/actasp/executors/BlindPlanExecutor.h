#pragma once


#include <actasp/PlanExecutor.h>

#include <stdexcept>
#include <list>
#include <map>
#include <actasp/Action.h>

namespace actasp {

class AspKR;

class Planner;

class Action;

class PlanningObserver;

class ExecutionObserver;

class BlindPlanExecutor : public PlanExecutor {

public:

  BlindPlanExecutor(AspKR &reasoner,
                    Planner &planner,
                    const std::map<std::string, Action *> &actionMap
  ) noexcept(false);

  using PlanExecutor::setGoal;

  void setGoal(const std::vector<actasp::AspRule> &goalRules) noexcept override;

  bool goalReached() const noexcept override {
    return isGoalReached;
  }

  bool failed() const noexcept override {
    return hasFailed;
  }

  void executeActionStep() override;

  void addExecutionObserver(ExecutionObserver &observer) noexcept override;

  void removeExecutionObserver(ExecutionObserver &observer) noexcept override;

  void addPlanningObserver(PlanningObserver &observer) noexcept;

  void removePlanningObserver(PlanningObserver &observer) noexcept;

  ~BlindPlanExecutor() override;


private:
  std::vector<actasp::AspRule> goalRules;
  bool isGoalReached;
  bool hasFailed;
  std::map<std::string, Action *> actionMap;

  std::list<std::unique_ptr<Action>> plan;
  unsigned int actionCounter;
  bool newAction;

  AspKR &kr;
  Planner &planner;

  std::list<std::reference_wrapper<ExecutionObserver>> executionObservers;
  std::list<std::reference_wrapper<PlanningObserver>> planningObservers;

  void computePlan();


};


}

