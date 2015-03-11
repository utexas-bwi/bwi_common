#ifndef actasp_ReplanningActionExecutor_h__guard
#define actasp_ReplanningActionExecutor_h__guard


#include <actasp/ActionExecutor.h>

#include <stdexcept>
#include <list>
#include <map>

namespace actasp {

class AspKR;
class Planner;
class Action;
class PlanningObserver;

class ReplanningActionExecutor : public ActionExecutor {

public:
	
	ReplanningActionExecutor(actasp::AspKR* reasoner, 
							 actasp::Planner *planner,
							 const std::map<std::string, Action * > &actionMap
							) throw (std::invalid_argument);
	
  using ActionExecutor::setGoal;
	void setGoal(const std::vector<actasp::AspRule>& goalRules) throw();

	bool goalReached() const throw() {
		return isGoalReached;
	}
	
	bool failed() const throw() {
		return hasFailed;
	}

	void executeActionStep();
  
  void addExecutionObserver(ExecutionObserver *observer) throw();
  void removeExecutionObserver(ExecutionObserver *observer) throw();
  
  void addPlanningObserver(PlanningObserver *observer) throw();
  void removePlanningObserver(PlanningObserver *observer) throw();
	
	~ReplanningActionExecutor();
	

private:
	std::vector<actasp::AspRule> goalRules;
	bool isGoalReached;
	bool hasFailed;
	std::map<std::string, Action * > actionMap;
	
	std::list<Action *> plan;
  unsigned int actionCounter;
  bool newAction;
	
	AspKR* kr;
	Planner *planner;
  
  std::list<ExecutionObserver*> executionObservers;
  std::list<PlanningObserver*> planningObservers;
  
  void computePlan();


};


}
#endif
