#include <actasp/executors/PartialPolicyExecutor.h>

#include <actasp/AspKR.h>
#include <actasp/MultiPlanner.h>
#include <actasp/AspRule.h>
#include <actasp/ActionSelector.h>
#include <actasp/Action.h>
#include <actasp/execution_observer_utils.h>
#include <actasp/action_utils.h>

#include <algorithm>
#include <iterator>
#include <functional>

using namespace std;

namespace actasp {

PartialPolicyExecutor::PartialPolicyExecutor(AspKR* kr, MultiPlanner *planner, ActionSelector *selector, 
                      const std::map<std::string, Action * >& actionMap, double suboptimality) :
                    
                    isGoalReached(false),
                    hasFailed(false),
                    actionCounter(0),
                    newAction(true),
                    active(NULL),
                    kr(kr),
                    planner(planner),
                    goalRules(),
                    policy(NULL),
                    suboptimality(suboptimality),
                    selector(selector),
                    actionMap(),
                    executionObservers() {

  transform(actionMap.begin(),actionMap.end(),inserter(this->actionMap,this->actionMap.end()),ActionMapDeepCopy());
}

PartialPolicyExecutor::~PartialPolicyExecutor() {
  delete active;
  for_each(actionMap.begin(),actionMap.end(),ActionMapDelete());
  delete policy;
}
  
  
void  PartialPolicyExecutor::setGoal(const std::vector<actasp::AspRule>& goalRules) throw() {

  this->goalRules = goalRules;

  isGoalReached = kr->currentStateQuery(goalRules).isSatisfied();

  if (!isGoalReached) {
    delete policy;
    policy = planner->computePolicy(goalRules,suboptimality);
    //TODO do the same for the other notifications, and get rid of helper classes?
    for_each(executionObservers.begin(),executionObservers.end(),bind2nd(mem_fun(&ExecutionObserver::policyChanged),policy));
  }

  hasFailed = (policy!= NULL) && (policy->empty());
  delete active;
  active = NULL;
  actionCounter = 0;
  newAction = true;
  
  for_each(executionObservers.begin(),executionObservers.end(),NotifyGoalChanged(goalRules));

}

bool PartialPolicyExecutor::goalReached() const throw() {
  return isGoalReached;
}
bool PartialPolicyExecutor::failed() const throw() {
  return hasFailed;
}

static Action *instantiateAction(const std::map<std::string, Action * >& actionMap, const AspFluent &actionFluent) {
  map<string, Action * >::const_iterator action = actionMap.find(actionFluent.getName());
  
  if(action == actionMap.end())
    throw logic_error("MultiPolicyExecutor: no action with name " + actionFluent.getName());
  
  return action->second->cloneAndInit(actionFluent);
}


void PartialPolicyExecutor::executeActionStep() {
  if (isGoalReached || hasFailed)
    return;
  
  if (active != NULL && !active->hasFinished()) {
    
    if (newAction) {
      for_each(executionObservers.begin(),executionObservers.end(),NotifyActionStart(active->toFluent(actionCounter)));
      newAction = false;
    } 
  
    active->run();

  } else {
    

    if (active != NULL) {
      for_each(executionObservers.begin(),executionObservers.end(),NotifyActionTermination(active->toFluent(actionCounter++)));
    }

    isGoalReached = kr->currentStateQuery(goalRules).isSatisfied();

    if (isGoalReached) //well done!
      return;

    //choose the next action
    AnswerSet currentState = kr->currentStateQuery(vector<AspRule>());
    set<AspFluent> state(currentState.getFluents().begin(), currentState.getFluents().end());
    ActionSet options = policy->actions(state);

    if (options.empty() || (active != NULL &&  active->hasFailed())) {
      //there's no action for this state, computing more plans

      //if the last action failed, we may want to have some more option

      PartialPolicy *otherPolicy = planner->computePolicy(goalRules,suboptimality);
      policy->merge(otherPolicy);
      delete otherPolicy;
      for_each(executionObservers.begin(),executionObservers.end(),bind2nd(mem_fun(&ExecutionObserver::policyChanged),policy));

      options = policy->actions(state);
      if (options.empty()) { //no actions available from here!
        hasFailed = true;
        return;
      }
    }

    set<AspFluent>::const_iterator chosen = selector->choose(options);

    delete active;
    active = instantiateAction(actionMap,*chosen);
    actionCounter++;
    newAction = true;


  }

}

void PartialPolicyExecutor::addExecutionObserver(ExecutionObserver *observer) throw() {
  executionObservers.push_back(observer);
}

void PartialPolicyExecutor::removeExecutionObserver(ExecutionObserver *observer) throw() {
  executionObservers.remove(observer);
}

}
