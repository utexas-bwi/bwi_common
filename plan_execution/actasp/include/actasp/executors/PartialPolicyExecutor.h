#ifndef actasp_MultiPolicyExecutor_h__guard
#define actasp_MultiPolicyExecutor_h__guard

#include <actasp/PlanExecutor.h>
#include <actasp/PartialPolicy.h>
#include <actasp/AspRule.h>

#include <vector>
#include <list>
#include <map>

namespace actasp {

class Action;

class AspKR;

class MultiPlanner;

class ActionSelector;

class ExecutionObserver;

class PlanningObserver;

class PartialPolicyExecutor : public PlanExecutor {
public:

  PartialPolicyExecutor(AspKR &kr, MultiPlanner &planner, ActionSelector &selector,
                        const std::map<std::string, Action *> &actionMap, double suboptimality);

  using PlanExecutor::setGoal;

  void setGoal(const std::vector<actasp::AspRule> &goalRules) noexcept;

  bool goalReached() const noexcept;

  bool failed() const noexcept;

  void executeActionStep();

  void addExecutionObserver(ExecutionObserver &observer) noexcept;

  void removeExecutionObserver(ExecutionObserver &observer) noexcept;

  ~PartialPolicyExecutor() override;

private:

  //state
  bool isGoalReached;
  bool hasFailed;
  unsigned int actionCounter;
  bool newAction;
  Action *active;

  //KR stuff
  AspKR &kr;
  MultiPlanner &planner;
  std::vector<actasp::AspRule> goalRules;

  PartialPolicy *policy;

  //customization
  double suboptimality;
  ActionSelector &selector;
  std::map<std::string, Action *> actionMap;

  //observers
  std::list<std::reference_wrapper<ExecutionObserver>> executionObservers;

};

}

#endif 
