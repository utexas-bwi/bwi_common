#pragma once

#include <actasp/asp/AspRule.h>
#include <actasp/AnswerSet.h>

#include <vector>
#include <list>
#include <stdexcept>

namespace actasp {

class Action;

struct Planner {

  virtual Plan computePlan(const std::vector<AspRule> &goal) const noexcept(false) = 0;

	virtual ~Planner() = default;
};
	
}

