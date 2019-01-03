#include <actasp/executors/ReplanningPlanExecutor.h>

#include <actasp/AspKR.h>
#include <actasp/ExecutionObserver.h>
#include <actasp/PlanningObserver.h>

#include <iostream>
#include <actasp/action_utils.h>
#include <actasp/execution_observer_utils.h>

using namespace std;

namespace actasp {

ReplanningPlanExecutor::ReplanningPlanExecutor(AspKR &reasoner,
                                               Planner &planner,
                                               const std::map<std::string, ActionFactory> &actionMap,
                                               ResourceManager &resourceManager
) noexcept(false) :
    goalRules(),
    isGoalReached(true),
    hasFailed(false),
    actionMap(actionMap),
    plan(),
    actionCounter(0),
    newAction(true),
    failureCount(0),
    kr(reasoner),
    planner(planner),
    resourceManager(resourceManager),
    executionObservers() {

}

ReplanningPlanExecutor::~ReplanningPlanExecutor() = default;

struct NotifyNewPlan {

  explicit NotifyNewPlan(AnswerSet plan) : plan(std::move(plan)) {}

  void operator()(PlanningObserver &observer) {
    observer.planChanged(plan);
  }

  AnswerSet plan;

};

void ReplanningPlanExecutor::computePlan() {
  isGoalReached = kr.currentStateQuery(goalRules).isSatisfied();

  if (!isGoalReached) {
    plan = planner.computePlan(goalRules).instantiateActions(actionMap, resourceManager);
    actionCounter = 0;
  } else {
    return;
  }

  hasFailed = plan.empty();

  if (!hasFailed)
    for_each(planningObservers.begin(), planningObservers.end(), NotifyNewPlan(planToAnswerSet(plan)));

}

void ReplanningPlanExecutor::setGoal(const std::vector<actasp::AspFluentRule> &goalRules) noexcept {
  this->goalRules = goalRules;

  for_each(executionObservers.begin(), executionObservers.end(), NotifyGoalChanged(goalRules));

  computePlan();

  failureCount = 0;
}

void ReplanningPlanExecutor::executeActionStep() {

  auto &current = plan.front();

  if (newAction) {
    for_each(executionObservers.begin(), executionObservers.end(),
             NotifyActionStart(current->toFluent(actionCounter)));
    newAction = false;
  }

  current->run();


  if (current->hasFinished()) {
    //destroy the action and pop a new one
    actionCounter += 1;
    auto as_fluent = current->toFluent(actionCounter);
    for_each(executionObservers.begin(), executionObservers.end(),
             [as_fluent, &current](ExecutionObserver &observer) {
               observer.actionTerminated(as_fluent, !current->hasFailed());
             });

    plan.pop_front();

    newAction = true;

    std::cout << "STARTING PLAN VERIFICATION. Remaining plan size: " << plan.size() << std::endl;


    if (!kr.isPlanValid(planToAnswerSet(plan), goalRules)) {

      //if (current->hasFailed()) {
      ++failureCount;
      //}
      cout << "Failed action count: " << failureCount << endl;

      if (failureCount >= 3) {
        std::cout << "FAILED TOO MANY TIMES. Aborting goal." << std::endl;
        for_each(executionObservers.begin(), executionObservers.end(),
                 [this, as_fluent](ExecutionObserver &observer) {
                   observer.planTerminated(ExecutionObserver::PlanStatus::TOO_MANY_ACTION_FAILURES,
                                           as_fluent, planToAnswerSet(plan));
                 });

        hasFailed = true;
        return;
      } else {
        std::cout << "PLAN VERIFICATION FAILED. Starting plan recomputation." << std::endl;

        //if not valid, replan
        plan.clear();

        computePlan();
      }
    } else if (plan.empty()) {
      computePlan();
    } else {
      failureCount = 0;
    }

    if (isGoalReached) {
      for_each(executionObservers.begin(), executionObservers.end(),
               [this, as_fluent](ExecutionObserver &observer) {
                 observer.planTerminated(ExecutionObserver::PlanStatus::SUCCEEDED,
                                         as_fluent, planToAnswerSet(plan));
               });
      return;
    }

    if (hasFailed) {
      for_each(executionObservers.begin(), executionObservers.end(),
               [this, as_fluent](ExecutionObserver &observer) {
                 observer.planTerminated(ExecutionObserver::PlanStatus::FAILED_TO_PLAN,
                                         as_fluent, planToAnswerSet(plan));
               });
      return;
    }

  }

}

void ReplanningPlanExecutor::addExecutionObserver(ExecutionObserver &observer) noexcept {
  executionObservers.emplace_back(observer);
}

void ReplanningPlanExecutor::removeExecutionObserver(ExecutionObserver &observer) noexcept {
  executionObservers.remove(observer);
}

void ReplanningPlanExecutor::addPlanningObserver(PlanningObserver &observer) noexcept {
  planningObservers.emplace_back(observer);
}

void ReplanningPlanExecutor::removePlanningObserver(PlanningObserver &observer) noexcept {
  planningObservers.remove(observer);
}

}
