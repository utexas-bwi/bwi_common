

#include <actasp/executors/ReplanningPlanExecutor.h>

#include <actasp/AspKR.h>
#include <actasp/AnswerSet.h>
#include <actasp/Planner.h>
#include <actasp/Action.h>
#include <actasp/action_utils.h>
#include <actasp/ExecutionObserver.h>
#include <actasp/PlanningObserver.h>
#include <actasp/execution_observer_utils.h>

#include <iostream>
#include <list>
#include <algorithm>
#include <iterator>

using namespace std;

namespace actasp {

ReplanningPlanExecutor::ReplanningPlanExecutor(actasp::AspKR* reasoner,
    actasp::Planner *planner,
    const std::map<std::string, Action * > &actionMap
                                                  ) noexcept(false) :
  goalRules(),
  isGoalReached(true),
  hasFailed(false),
  actionMap(),
  plan(),
  actionCounter(0),
  newAction(true),
  failedActionCount(0),
  kr(reasoner),
  planner(planner),
  executionObservers(){
  if (reasoner == nullptr)
    throw invalid_argument("ReplanningPlanExecutor: reasoner is NULL");

  if (planner == nullptr)
    throw invalid_argument("ReplanningPlanExecutor: planner is NULL");

  transform(actionMap.begin(),actionMap.end(),inserter(this->actionMap,this->actionMap.begin()),ActionMapDeepCopy());
}

ReplanningPlanExecutor::~ReplanningPlanExecutor() {
  for_each(actionMap.begin(),actionMap.end(),ActionMapDelete());
}

struct NotifyNewPlan {

  NotifyNewPlan(const AnswerSet& plan) : plan(plan) {}

  void operator()(PlanningObserver* observer) {
    observer->planChanged(plan);
  }

  AnswerSet plan;

};

void ReplanningPlanExecutor::computePlan() {
  isGoalReached = kr->currentStateQuery(goalRules).isSatisfied();

  if (!isGoalReached) {
    plan = planner->computePlan(goalRules).instantiateActions(actionMap);
    actionCounter = 0;
  }

  hasFailed = plan.empty();

  if(!hasFailed)
    for_each(planningObservers.begin(),planningObservers.end(),NotifyNewPlan(planToAnswerSet(plan)));

}

void ReplanningPlanExecutor::setGoal(const std::vector<actasp::AspRule>& goalRules) noexcept {
  this->goalRules = goalRules;

  for_each(executionObservers.begin(),executionObservers.end(),NotifyGoalChanged(goalRules));

  computePlan();

  failedActionCount = 0;
}




void ReplanningPlanExecutor::executeActionStep() {

  if (isGoalReached || hasFailed)
    return;


  Action::Ptr current = plan.front();

  if(newAction) {
      for_each(executionObservers.begin(),executionObservers.end(),NotifyActionStart(current->toFluent(actionCounter)));
      newAction = false;
  }


  current->run();

  if (current->hasFinished()) {
    //destroy the action and pop a new one

    for_each(executionObservers.begin(),executionObservers.end(),NotifyActionTermination(current->toFluent(actionCounter++)));
    
    plan.pop_front();

    newAction = true;

    std::cout << "STARTING PLAN VERIFICATION. Remaining plan size: " << plan.size() << std::endl;

    if (plan.empty() || !kr->isPlanValid(planToAnswerSet(plan),goalRules)) {

      if (current->hasFailed()) {
        ++failedActionCount;
      }

      if (failedActionCount >= 3) {
        std::cout << "FAILED TOO MANY TIMES. Aborting goal." << std::endl;
        hasFailed = true;
      }
      else {
        std::cout << "PLAN VERIFICATION FAILED. Starting plan recomputation." << std::endl;

        //if not valid, replan
        plan.clear();

        computePlan();
      }
      
    }
    else {
      failedActionCount = 0;
    }

  }



}

void ReplanningPlanExecutor::addExecutionObserver(ExecutionObserver *observer) noexcept {
  executionObservers.push_back(observer);
}

void ReplanningPlanExecutor::removeExecutionObserver(ExecutionObserver *observer) noexcept {
  executionObservers.remove(observer);
}

void ReplanningPlanExecutor::addPlanningObserver(PlanningObserver *observer) noexcept {
  planningObservers.push_back(observer);
}

void ReplanningPlanExecutor::removePlanningObserver(PlanningObserver *observer) noexcept {
  planningObservers.remove(observer);
}

}
