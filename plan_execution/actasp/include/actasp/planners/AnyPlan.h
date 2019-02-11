#pragma once

#include <actasp/Planner.h>


namespace actasp {
	
class MultiPlanner;

class AnyPlan : public Planner {
public:
	
	//doesn't own
	AnyPlan(actasp::MultiPlanner *actualPlanner, double suboptimality = 1.);

	Plan computePlan(const std::vector<actasp::AspRule> &goal) const override;
private:
	MultiPlanner *actualPlanner;
  double suboptimality;
	
};

}



