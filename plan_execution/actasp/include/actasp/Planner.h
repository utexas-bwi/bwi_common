#ifndef actasp_Planner_h__guard
#define actasp_Planner_h__guard

#include <actasp/AspRule.h>
#include <actasp/AnswerSet.h>

#include <vector>
#include <list>
#include <stdexcept>

namespace actasp {

class Action;

struct Planner {

	virtual AnswerSet computePlan(const std::vector<actasp::AspRule>& goal) const throw (std::logic_error) =0;

	virtual ~Planner() {}
};
	
}
#endif
