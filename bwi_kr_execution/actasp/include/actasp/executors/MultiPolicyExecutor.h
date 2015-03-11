#ifndef actasp_MultiPolicyExecutor_h__guard
#define actasp_MultiPolicyExecutor_h__guard

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

class MultiPolicyExecutor : public ActionExecutor {
public:
	
  MultiPolicyExecutor(AspKR* kr, MultiPlanner *planner, ActionSelector *selector, 
                      const std::map<std::string, Action * >& actionMap, double suboptimality);
  
  using ActionExecutor::setGoal;
  void setGoal(const std::vector<actasp::AspRule>& goalRules) throw();

	bool goalReached() const throw();
	bool failed() const throw();

	void executeActionStep();
  
  void addExecutionObserver(ExecutionObserver *observer) throw();
  void removeExecutionObserver(ExecutionObserver *observer) throw();
  
  ~MultiPolicyExecutor();
 
private:
  
	//state
  bool isGoalReached;
	bool hasFailed;
  unsigned int actionCounter;
  bool newAction;
  Action *active;
  
  //KR stuff
  AspKR* kr;
  MultiPlanner *planner;
	std::vector<actasp::AspRule> goalRules;
  
	MultiPolicy policy;

  //customization
  double suboptimality;
  ActionSelector *selector;
  std::map<std::string, Action * > actionMap;
  
  //observers
  std::list<ExecutionObserver*> executionObservers;

};

}

#endif 
