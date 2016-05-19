#ifndef actasp_RLActionExecutor_h__guard
#define actasp_RLActionExecutor_h__guard

#include <actasp/ActionExecutor.h>
#include <actasp/MultiPolicy.h>
#include <actasp/AspRule.h>

#include <vector>
#include <list>

namespace actasp {
  
class AspKR;
class MultiPlanner;
class ActionSelector;
class ExecutionObserver;
class PlanningObserver;
class Action;

class RLActionExecutor : public ActionExecutor {
public:
  
  RLActionExecutor(AspKR* kr, ActionSelector *selector, const std::map<std::string, Action * >& actionMap);
  
  using ActionExecutor::setGoal;
  void setGoal(const std::vector<actasp::AspRule>& goalRules) throw();

  bool goalReached() const throw();
  bool failed() const throw();

  void executeActionStep();
  
  void addExecutionObserver(ExecutionObserver *observer) throw();
  void removeExecutionObserver(ExecutionObserver *observer) throw();
  
  ~RLActionExecutor();
 
private:
  
  //state
  bool isGoalReached;
  unsigned int actionCounter;
  bool newAction;
  Action *active;
  
  //KR stuff
  AspKR* kr;
  std::vector<actasp::AspRule> goalRules;

  //customization
  ActionSelector *selector;
  std::map<std::string, Action * > actionMap;
  
  //observers
  std::list<ExecutionObserver*> executionObservers;

};

}

#endif 
