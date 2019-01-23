#pragma once

#include <actasp/asp/AspRule.h>
#include <vector>

namespace actasp {
  
class ExecutionObserver;

struct PlanExecutor {

	void setGoal(const AspRule& goalRule) noexcept {
		std::vector<AspRule> goal;
		goal.push_back(goalRule);
		this->setGoal(goal);
	}
	
	virtual void setGoal(const std::vector<actasp::AspRule>& goalRules) noexcept = 0;

	virtual bool goalReached() const noexcept =0;
	virtual bool failed() const noexcept = 0;

	virtual void executeActionStep() = 0;

  virtual void addExecutionObserver(ExecutionObserver &observer) noexcept = 0;

  virtual void removeExecutionObserver(ExecutionObserver &observer) noexcept = 0;

  virtual ~PlanExecutor() = default;
};

}


