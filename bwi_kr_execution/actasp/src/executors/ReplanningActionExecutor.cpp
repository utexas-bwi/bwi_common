

#include <actasp/executors/ReplanningActionExecutor.h>

#include <actasp/AspKR.h>
#include <actasp/AnswerSet.h>
#include <actasp/Planner.h>
#include <actasp/Action.h>
#include <actasp/action_utils.h>
#include <actasp/ExecutionObserver.h>
#include <actasp/PlanningObserver.h>
#include <actasp/execution_observer_utils.h>

#include <list>
#include <algorithm>
#include <iterator>

using namespace std;

namespace actasp {

ReplanningActionExecutor::ReplanningActionExecutor(actasp::AspKR* reasoner,
    actasp::Planner *planner,
    const std::map<std::string, Action * > &actionMap
                                                  ) throw (std::invalid_argument) :
  goalRules(),
  isGoalReached(true),
  hasFailed(false),
  actionMap(),
  plan(),
  actionCounter(0),
  newAction(true),
  kr(reasoner),
  planner(planner),
  executionObservers(){
  if (reasoner == NULL)
    throw invalid_argument("ReplanningActionExecutor: reasoner is NULL");

  if (planner == NULL)
    throw invalid_argument("ReplanningActionExecutor: planner is NULL");

  transform(actionMap.begin(),actionMap.end(),inserter(this->actionMap,this->actionMap.begin()),ActionMapDeepCopy());
}

ReplanningActionExecutor::~ReplanningActionExecutor() {
  for_each(actionMap.begin(),actionMap.end(),ActionMapDelete());
}

struct NotifyNewPlan {
  
  NotifyNewPlan(const AnswerSet& plan) : plan(plan) {}
  
  void operator()(PlanningObserver* observer) {
    observer->planChanged(plan);
  }
  
  AnswerSet plan;
  
};

void ReplanningActionExecutor::computePlan() {
  isGoalReached = kr->currentStateQuery(goalRules).isSatisfied();

  if (!isGoalReached) {
    plan = planner->computePlan(goalRules).instantiateActions(actionMap);
    actionCounter = 0;
  }

  hasFailed = plan.empty();
  
  if(!hasFailed)
    for_each(planningObservers.begin(),planningObservers.end(),NotifyNewPlan(planToAnswerSet(plan)));
  
}

void ReplanningActionExecutor::setGoal(const std::vector<actasp::AspRule>& goalRules) throw() {
  this->goalRules = goalRules;

  computePlan();
}




void ReplanningActionExecutor::executeActionStep() {

  if (isGoalReached || hasFailed)
    return;


  Action *current = plan.front();

  if(newAction) {
      for_each(executionObservers.begin(),executionObservers.end(),NotifyActionStart(current->toFluent(actionCounter)));
      newAction = false;
  }
 

  current->run();

  if (current->hasFinished()) {
    //destroy the action and pop a new one
    
    for_each(executionObservers.begin(),executionObservers.end(),NotifyActionTermination(current->toFluent(actionCounter++)));
    
    delete current;
    plan.pop_front();
    
    newAction = true;

    if (plan.empty() || !kr->isPlanValid(planToAnswerSet(plan),goalRules)) {
      
      //if not valid, replan
      for_each(plan.begin(),plan.end(),ActionDeleter());
      plan.clear();

      computePlan();

    }

  }
  
  
  
}

void ReplanningActionExecutor::addExecutionObserver(ExecutionObserver *observer) throw() {
  executionObservers.push_back(observer);
}

void ReplanningActionExecutor::removeExecutionObserver(ExecutionObserver *observer) throw() {
  executionObservers.remove(observer);
}

void ReplanningActionExecutor::addPlanningObserver(PlanningObserver *observer) throw() {
  planningObservers.push_back(observer);
}

void ReplanningActionExecutor::removePlanningObserver(PlanningObserver *observer) throw() {
  planningObservers.remove(observer);
}

}
