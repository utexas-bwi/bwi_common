#include <actasp/executors/MultiPolicyExecutor.h>

#include <actasp/AspKR.h>
#include <actasp/MultiPlanner.h>
#include <actasp/AspRule.h>
#include <actasp/ActionSelector.h>
#include <actasp/Action.h>
#include <actasp/execution_observer_utils.h>
#include <actasp/action_utils.h>

#include <algorithm>
#include <iterator>

#include <ros/ros.h>

using namespace std;

namespace actasp {

MultiPolicyExecutor::MultiPolicyExecutor(AspKR* kr, MultiPlanner *planner, ActionSelector *selector, 
                      const std::map<std::string, Action * >& actionMap, double suboptimality) :
                    
                    isGoalReached(false),
                    hasFailed(false),
                    actionCounter(0),
                    newAction(true),
                    active(NULL),
                    kr(kr),
                    planner(planner),
                    goalRules(),
                    policy(actionMapToSet(actionMap)),
                    suboptimality(suboptimality),
                    selector(selector),
                    actionMap(),
                    executionObservers() {

  transform(actionMap.begin(),actionMap.end(),inserter(this->actionMap,this->actionMap.end()),ActionMapDeepCopy());
}

MultiPolicyExecutor::~MultiPolicyExecutor() {
  delete active;
  for_each(actionMap.begin(),actionMap.end(),ActionMapDelete());
}
  
  
void  MultiPolicyExecutor::setGoal(const std::vector<actasp::AspRule>& goalRules) throw() {

  this->goalRules = goalRules;

  isGoalReached = kr->currentStateQuery(goalRules).isSatisfied();

  if (!isGoalReached)
    policy = planner->computePolicy(goalRules,suboptimality);

  hasFailed = policy.empty();
  delete active;
  active = NULL;
  actionCounter = 0;
  newAction = true;

}

bool MultiPolicyExecutor::goalReached() const throw() {
  return isGoalReached;
}
bool MultiPolicyExecutor::failed() const throw() {
  return hasFailed;
}

static Action *instantiateAction(const std::map<std::string, Action * >& actionMap, const AspFluent &actionFluent) {
  map<string, Action * >::const_iterator action = actionMap.find(actionFluent.getName());
  
  if(action == actionMap.end())
    throw logic_error("MultiPolicyExecutor: no action with name " + actionFluent.getName());
  
  return action->second->cloneAndInit(actionFluent);
}


void MultiPolicyExecutor::executeActionStep() {
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
    ActionSet options = policy.actions(state);

    if (options.empty() || (active != NULL &&  active->hasFailed())) {
      //there's no action for this state, computing more plans

      //if the last action failed, we may want to have some more options
      

      MultiPolicy otherPolicy = planner->computePolicy(goalRules,suboptimality);
      policy.merge(otherPolicy);

      options = policy.actions(state);
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

void MultiPolicyExecutor::addExecutionObserver(ExecutionObserver *observer) throw() {
  executionObservers.push_back(observer);
}

void MultiPolicyExecutor::removeExecutionObserver(ExecutionObserver *observer) throw() {
  executionObservers.remove(observer);
}

}
