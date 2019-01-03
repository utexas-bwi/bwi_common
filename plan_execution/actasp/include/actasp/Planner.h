#pragma once

#include <actasp/AspRule.h>
#include <actasp/AnswerSet.h>

#include <vector>
#include <list>
#include <stdexcept>

namespace actasp {

class Action;

struct Planner {

	virtual AnswerSet computePlan(const std::vector<actasp::AspFluentRule>& goal) const noexcept(false) =0;

	virtual ~Planner() = default;
};
	
}

