#pragma once

#include <actasp/Planner.h>


namespace actasp {
	
class MultiPlanner;

class AnyPlan : public Planner {
public:
	
	//doesn't own
	AnyPlan(actasp::MultiPlanner *actualPlanner, double suboptimality = 1.);
	
	AnswerSet computePlan(const std::vector<actasp::AspRule>& goal) const;
private:
	MultiPlanner *actualPlanner;
  double suboptimality;
	
};

}



