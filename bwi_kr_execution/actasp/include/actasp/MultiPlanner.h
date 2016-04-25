
#ifndef actasp_MultiPlanner_h__guard
#define actasp_MultiPlanner_h__guard

#include <actasp/Planner.h>
#include <actasp/AspRule.h>
#include <actasp/MultiPolicy.h>
#include <actasp/GraphPolicy.h>

#include <vector>
#include <list>

namespace actasp {

class Action;

struct MultiPlanner : public actasp::Planner {

virtual std::vector< AnswerSet > computeAllPlans(const std::vector<actasp::AspRule>& goal, double suboptimality) const throw (std::logic_error)=0;

virtual PartialPolicy* computePolicy(const std::vector<actasp::AspRule>& goal, double suboptimality) const throw (std::logic_error)=0;

virtual ~MultiPlanner(){}

};

}

#endif
