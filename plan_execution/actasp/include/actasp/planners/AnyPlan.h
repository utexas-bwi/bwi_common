
#ifndef actasp_AnyPlan_h__guard
#define actasp_AnyPlan_h__guard

#include <actasp/Planner.h>


namespace actasp {
	
class MultiPlanner;

class AnyPlan : public Planner {
public:
	
	//doesn't own
	AnyPlan(actasp::MultiPlanner *actualPlanner, double suboptimality = 1.);
	
	AnswerSet computePlan(const std::vector<actasp::AspRule>& goal) const throw (std::logic_error);
private:
	MultiPlanner *actualPlanner;
  double suboptimality;
	
};

}


#endif
