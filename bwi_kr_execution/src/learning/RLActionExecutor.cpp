#include "RLActionExecutor.h"

#include <actasp/AspKR.h>
#include <actasp/AspRule.h>
#include <actasp/ActionSelector.h>
#include <actasp/Action.h>
#include <actasp/execution_observer_utils.h>
#include <actasp/action_utils.h>

#include <algorithm>
#include <iterator>

using namespace std;

namespace actasp {

RLActionExecutor::RLActionExecutor(AspKR* kr, ActionSelector *selector, 
                      const std::map<std::string, Action * >& actionMap) :
                    
                    isGoalReached(false),
                    actionCounter(0),
                    newAction(true),
                    active(NULL),
                    kr(kr),
                    goalRules(),
                    selector(selector),
                    actionMap(),
                    executionObservers() {

  transform(actionMap.begin(),actionMap.end(),inserter(this->actionMap,this->actionMap.end()),ActionMapDeepCopy());
}

RLActionExecutor::~RLActionExecutor() {
  delete active;
  for_each(actionMap.begin(),actionMap.end(),ActionMapDelete());
}
  
  
void  RLActionExecutor::setGoal(const std::vector<actasp::AspRule>& goalRules) throw() {

  this->goalRules = goalRules;

  isGoalReached = kr->currentStateQuery(goalRules).isSatisfied();

  delete active;
  active = NULL;
  actionCounter = 0;
  newAction = true;

}

bool RLActionExecutor::goalReached() const throw() {
  return isGoalReached;
}

bool RLActionExecutor::failed() const throw() {
  return false;
}

static Action *instantiateAction(const std::map<std::string, Action * >& actionMap, const AspFluent &actionFluent) {
  map<string, Action * >::const_iterator action = actionMap.find(actionFluent.getName());
  
  if(action == actionMap.end())
    throw logic_error("RLActionExecutor: no action with name " + actionFluent.getName());
  
  return action->second->cloneAndInit(actionFluent);
}


void RLActionExecutor::executeActionStep() {
  if (isGoalReached)
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
    
    ActionSet options = kr->availableActions();
    
    set<AspFluent>::const_iterator chosen = selector->choose(options);

    delete active;
    active = instantiateAction(actionMap,*chosen);
    actionCounter++;
    newAction = true;

  }

}

void RLActionExecutor::addExecutionObserver(ExecutionObserver *observer) throw() {
  executionObservers.push_back(observer);
}

void RLActionExecutor::removeExecutionObserver(ExecutionObserver *observer) throw() {
  executionObservers.remove(observer);
}

}
