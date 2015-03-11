#ifndef actasp_ActionExecutor_h__guard
#define actasp_ActionExecutor_h__guard

#include <actasp/AspRule.h>
#include <vector>

namespace actasp {
  
class ExecutionObserver;

struct ActionExecutor {

	void setGoal(const AspRule& goalRule) throw() {
		std::vector<AspRule> goal;
		goal.push_back(goalRule);
		this->setGoal(goal);
	}
	
	virtual void setGoal(const std::vector<actasp::AspRule>& goalRules) throw() = 0;

	virtual bool goalReached() const throw() =0;
	virtual bool failed() const throw() = 0;

	virtual void executeActionStep() = 0;
  
  virtual void addExecutionObserver(ExecutionObserver *observer) throw() = 0;
  virtual void removeExecutionObserver(ExecutionObserver *observer) throw() =0;

	virtual ~ActionExecutor() {}
};

}

#endif
